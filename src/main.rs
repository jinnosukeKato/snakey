use std::ops::Range;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use esp_idf_svc::hal::{
    i2s::config::{
        Config, DataBitWidth, PdmRxClkConfig, PdmRxConfig, PdmRxGpioConfig, PdmRxSlotConfig,
        SlotMode,
    },
    i2s::I2sDriver,
    peripherals::Peripherals,
    units::Hertz,
};
use pitch_detector::{note::detect_note_in_range, pitch::HannedFftDetector};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

mod keyboard;
use keyboard::Keyboard;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("Startup Snakey!");

    // BLEキーボードの初期化
    let mut keyboard = Keyboard::new().expect("Failed to initialize BLE keyboard");
    log::info!("BLE keyboard initialized, waiting for connection...");

    // 人間の音声，笛に適した検出器として，Hann窓をかけたFFTベースのピッチ検出器を用いる
    let mut detector = HannedFftDetector::default();

    // ペリフェラルの取得
    let peripherals = Peripherals::take().expect("Failed to take peripherals");

    // OLEDディスプレイの初期化
    let i2c = esp_idf_svc::hal::i2c::I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio5,
        peripherals.pins.gpio6,
        &esp_idf_svc::hal::i2c::config::Config::default().baudrate(Hertz(400_000)),
    )
    .expect("Failed to create I2C driver");

    let i2c_disp_if = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(i2c_disp_if, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().expect("Failed to initialize display");

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    // PDMマイクの初期化
    const SAMPLE_RATE: u32 = 16_000;
    let mut pdm_driver = I2sDriver::new_pdm_rx(
        peripherals.i2s0,
        &PdmRxConfig::new(
            Config::default(),
            PdmRxClkConfig::from_sample_rate_hz(SAMPLE_RATE),
            PdmRxSlotConfig::from_bits_per_sample_and_slot_mode(
                DataBitWidth::Bits16,
                SlotMode::Mono,
            ),
            PdmRxGpioConfig::default(),
        ),
        // pin assign; ref. https://wiki.seeedstudio.com/xiao_esp32s3_sense_mic/
        peripherals.pins.gpio42, // PDM_CLK
        peripherals.pins.gpio41, // PDM_DATA
    )
    .expect("Failed to create I2S driver");

    pdm_driver
        .rx_enable()
        .expect("Failed to enable I2S RX channel");

    // バッファの設定
    const BUFFER_SIZE: usize = 1024 * 4; // 4096 バイト = 2048 サンプル (16kHzで約128ms)
    let mut buffer = vec![0u8; BUFFER_SIZE]; // スタックオーバーフローを避けるためVecでヒープ領域（RAM）に確保

    log::info!("Initialization complete, entering main loop...");

    // メインループ
    loop {
        if !keyboard.connected() {
            log::info!("Waiting for BLE connection...");
            esp_idf_svc::hal::delay::Ets::delay_ms(500);
            continue;
        }

        // バッファがいっぱいになるまで継続的にチャンクを受信する
        let mut total_read = 0;
        while total_read < BUFFER_SIZE {
            let bytes_read = pdm_driver
                .read(
                    &mut buffer[total_read..],
                    esp_idf_svc::hal::delay::BLOCK.into(),
                )
                .expect("Failed to read from I2S");
            total_read += bytes_read;
        }

        // 受信したバッファのデータを16bit PCMとして解釈し，f64のベクタに変換
        let mut buff_f64 = buffer
            .chunks_exact(2)
            .map(|chunk| {
                // PDMの音声データが内部チップで16bit PCMに変換されているので，
                // 16bit(2バイト)のリトルエンディアンPCMデータとして解釈してf64に変換
                let val = i16::from_le_bytes([chunk[0], chunk[1]]) as f64;
                // XIAO ESP32S3 SenseのMEMSマイクは音が小さい傾向があるためx4倍のゲインをかけた上，一応[-1.0, 1.0] に正規化
                (val * 4.0) / 32768.0
            })
            .collect::<Vec<f64>>();

        // PDMマイク特有のDCオフセットを除去
        let mean = buff_f64.iter().sum::<f64>() / buff_f64.len() as f64;
        for x in &mut buff_f64 {
            *x -= mean;
        }

        // マイクの入力レベル確認用RMS 範囲 [-1.0, 1.0]
        let rms = (buff_f64.iter().map(|&x| x * x).sum::<f64>() / buff_f64.len() as f64).sqrt();
        // log::info!("RMS: {:.4}", rms);

        const THRESHOLD: f64 = 0.01; // 無音判断用RMS閾値
        if rms > THRESHOLD {
            match detect_note_in_range(
                &buff_f64,
                &mut detector,
                SAMPLE_RATE as f64,
                Range {
                    start: 60.0,
                    end: 2000.0,
                },
            ) {
                Some(note) => {
                    log::info!(
                        "Detected note: {}{}, freq: {:.2}Hz",
                        note.note_name,
                        note.octave,
                        note.actual_freq
                    );

                    display
                        .clear(BinaryColor::Off)
                        .expect("Failed to clear display");
                    let disp_text = format!(
                        "{:<2}{}, {:.2}Hz",
                        note.note_name.to_string(),
                        note.octave,
                        note.actual_freq
                    );
                    Text::with_baseline(&disp_text, Point::zero(), text_style, Baseline::Top)
                        .draw(&mut display)
                        .expect("Failed to draw text");
                    display.flush().expect("Failed to flush text to display");

                    keyboard.write(&format!("{}{}", note.note_name, note.octave));
                }
                None => {
                    log::error!("Pitch was not detected");
                }
            }
            esp_idf_svc::hal::delay::Ets::delay_ms(50);
        } else {
            // log::info!("Silence...");
        }
    }
}
