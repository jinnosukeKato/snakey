use esp_idf_svc::hal::{
    i2s::config::{
        Config, DataBitWidth, PdmRxClkConfig, PdmRxConfig, PdmRxGpioConfig, PdmRxSlotConfig,
        SlotMode,
    },
    i2s::I2sDriver,
    peripherals::Peripherals,
};

fn main() {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, world!");

    let peripherals = Peripherals::take().expect("Failed to take peripherals");

    let mut pdm_driver = I2sDriver::new_pdm_rx(
        peripherals.i2s0,
        &PdmRxConfig::new(
            Config::default(),
            PdmRxClkConfig::from_sample_rate_hz(16_000),
            PdmRxSlotConfig::from_bits_per_sample_and_slot_mode(
                DataBitWidth::Bits16,
                SlotMode::Mono,
            ),
            PdmRxGpioConfig::default(),
        ),
        peripherals.pins.gpio40,
        peripherals.pins.gpio41,
    )
    .expect("Failed to create I2S driver");

    pdm_driver
        .rx_enable()
        .expect("Failed to enable I2S RX channel");

    let mut buffer = [0u8; 1024 * 4];
    loop {
        pdm_driver
            .read(&mut buffer, 1000)
            .expect("Failed to read from I2S");
        log::info!("Read {} samples", buffer.len());
    }
}
