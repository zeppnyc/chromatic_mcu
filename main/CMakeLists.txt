cmake_minimum_required(VERSION 3.22)

idf_component_register(
    SRCS
        "main.c" "gfx.c" "fpga_tx.c" "fpga_rx.c" "fpga_common.c" "pwrmgr.c"
        "fusion_savestate.c" "fpga_state_port.c" "savestate_storage.c"
        "fusion_savestate_smoke48a.c"
        "fusion_savestate_transport_probe.c"
    INCLUDE_DIRS "."
    REQUIRES
        images esp_driver_uart esp_driver_spi esp_driver_gpio nvs_flash fatfs
        common battery button osd menu_mgr settings mutex esp_app_format
        console crc
)
