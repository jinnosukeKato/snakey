use std::ops::Range;

use esp_idf_svc::hal::{
    i2s::config::{
        Config, DataBitWidth, PdmRxClkConfig, PdmRxConfig, PdmRxGpioConfig, PdmRxSlotConfig,
        SlotMode,
    },
    i2s::I2sDriver,
    peripherals::Peripherals,
};
use pitch_detector::{
    note::{detect_note, detect_note_in_range},
    pitch::PowerCepstrum,
};

fn main() {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Startup");

    let peripherals = Peripherals::take().expect("Failed to take peripherals");

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

    const BUFFER_SIZE: usize = 1024 * 4;
    let mut buffer = [0u8; BUFFER_SIZE];
    let mut detector = PowerCepstrum::default();
    loop {
        let bytes_read = pdm_driver
            .read(&mut buffer, esp_idf_svc::hal::delay::BLOCK.into())
            .expect("Failed to read from I2S");

        if bytes_read == 0 {
            println!("Zero bytes read!");
            continue;
        }

        // 16bit(2バイト)のリトルエンディアンPCMデータとして解釈してf64に変換
        // PDMの音声が内部チップで16bit PCMに変換されている...はず
        let mut buff_f64 = buffer[..bytes_read]
            .chunks_exact(2)
            .map(|chunk| i16::from_le_bytes([chunk[0], chunk[1]]) as f64)
            .collect::<Vec<f64>>();

        // PDMマイク特有の巨大なDCオフセット(直流成分)を除去
        let mean = buff_f64.iter().sum::<f64>() / buff_f64.len() as f64;
        for x in &mut buff_f64 {
            *x -= mean;
        }

        // AC成分のRMSを計算
        let rms = (buff_f64.iter().map(|&x| x * x).sum::<f64>() / buff_f64.len() as f64).sqrt();

        // 実際のRMS値を表示して、マイクの入力レベルを確認します
        // 音を出した時と無音時の数値を比較して、下の threshold を決めてください
        println!("RMS: {:.2}", rms);
        let threshold = 100.0; // 表示されたRMS値を参考にここを調整

        if rms > threshold {
            match detect_note_in_range(
                &buff_f64,
                &mut detector,
                SAMPLE_RATE as f64,
                Range {
                    start: 20.0,
                    end: 5000.0,
                },
            ) {
                Some(note) => println!("Detected freq: {:?}", note.actual_freq),
                None => println!("No note detected"),
            }
        } else {
            println!("Silence...");
        }
    }
}
