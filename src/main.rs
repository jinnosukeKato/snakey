use std::ops::Range;

use esp_idf_svc::hal::{
    i2s::config::{
        Config, DataBitWidth, PdmRxClkConfig, PdmRxConfig, PdmRxGpioConfig, PdmRxSlotConfig,
        SlotMode,
    },
    i2s::I2sDriver,
    peripherals::Peripherals,
};
use pitch_detector::{note::detect_note_in_range, pitch::HannedFftDetector};

mod keyboard;
use keyboard::Keyboard;

fn main() {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Startup");

    let peripherals = Peripherals::take().expect("Failed to take peripherals");

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
        peripherals.pins.gpio42, // PDM_CLK
        peripherals.pins.gpio41, // PDM_DATA
    )
    .expect("Failed to create I2S driver");

    pdm_driver
        .rx_enable()
        .expect("Failed to enable I2S RX channel");

    // BLEキーボードの初期化
    let mut keyboard = Keyboard::new().expect("Failed to initialize BLE keyboard");

    // バッファの設定
    const BUFFER_SIZE: usize = 1024 * 4; // 4096 バイト = 2048 サンプル (16kHzで約128ms)
    let mut buffer = vec![0u8; BUFFER_SIZE]; // スタックオーバーフローを避けるためVecでヒープ領域（RAM）に確保

    // 人間の音声，笛に適した検出器として，Hann窓をかけたFFTベースのピッチ検出器を用いる
    let mut detector = HannedFftDetector::default();

    // メインループ
    loop {
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

        // 16bit(2バイト)のリトルエンディアンPCMデータとして解釈してf64に変換
        // PDMの音声データが内部チップで16bit PCMに変換されている
        let mut buff_f64 = buffer
            .chunks_exact(2)
            .map(|chunk| {
                // XIAO ESP32S3のマイクは音が小さい傾向があるため、入力時点でデジタル的にゲインをかける(x4倍)
                // 各種ピッチ計算ライブラリが想定しやすいように、最大振幅を基準に [-1.0, 1.0] に正規化
                let val = i16::from_le_bytes([chunk[0], chunk[1]]) as f64;
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
        println!("RMS: {:.4}", rms);

        let threshold = 0.01; // 無音判断用RMS閾値
        if rms > threshold {
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
                    println!(
                        "Detected note: {:?}, freq: {:.2}Hz",
                        note.note_name, note.actual_freq
                    );
                    keyboard.write(&format!("{}", note.note_name));
                }
                None => {
                    println!("Pitch was not detected")
                }
            }
        } else {
            println!("Silence...");
        }
    }
}
