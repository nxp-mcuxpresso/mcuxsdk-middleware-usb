include_guard(GLOBAL)


if (CONFIG_USE_middleware_usb_phydcd_config_header)
# Add set(CONFIG_USE_middleware_usb_phydcd_config_header true) in config.cmake to use this component

message("middleware_usb_phydcd_config_header component is included from ${CMAKE_CURRENT_LIST_FILE}.")

add_config_file(${CMAKE_CURRENT_LIST_DIR}/./output/npw/dcd_config/phydcd/usb_phydcd_config.h ${CMAKE_CURRENT_LIST_DIR}/./output/npw/dcd_config/phydcd middleware_usb_phydcd_config_header)


endif()


if (CONFIG_USE_middleware_usb_hsdcd_config_header)
# Add set(CONFIG_USE_middleware_usb_hsdcd_config_header true) in config.cmake to use this component

message("middleware_usb_hsdcd_config_header component is included from ${CMAKE_CURRENT_LIST_FILE}.")

add_config_file(${CMAKE_CURRENT_LIST_DIR}/./output/npw/dcd_config/hsdcd/usb_hsdcd_config.h ${CMAKE_CURRENT_LIST_DIR}/./output/npw/dcd_config/hsdcd middleware_usb_hsdcd_config_header)


endif()


if (CONFIG_USE_middleware_usb_device_khci_config_header)
# Add set(CONFIG_USE_middleware_usb_device_khci_config_header true) in config.cmake to use this component

message("middleware_usb_device_khci_config_header component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if((CONFIG_DEVICE_ID STREQUAL MCXC143 OR CONFIG_DEVICE_ID STREQUAL MCXC144 OR CONFIG_DEVICE_ID STREQUAL MCXC243 OR CONFIG_DEVICE_ID STREQUAL MCXC244 OR CONFIG_DEVICE_ID STREQUAL MCXC443 OR CONFIG_DEVICE_ID STREQUAL MCXC444 OR CONFIG_DEVICE_ID STREQUAL MCXC141 OR CONFIG_DEVICE_ID STREQUAL MCXC142 OR CONFIG_DEVICE_ID STREQUAL MCXC242 OR CONFIG_DEVICE_ID STREQUAL MCXN546 OR CONFIG_DEVICE_ID STREQUAL MCXN547 OR CONFIG_DEVICE_ID STREQUAL MCXN946 OR CONFIG_DEVICE_ID STREQUAL MCXN947 OR CONFIG_DEVICE_ID STREQUAL MCXA132 OR CONFIG_DEVICE_ID STREQUAL MCXA133 OR CONFIG_DEVICE_ID STREQUAL MCXA142 OR CONFIG_DEVICE_ID STREQUAL MCXA143 OR CONFIG_DEVICE_ID STREQUAL MCXA144 OR CONFIG_DEVICE_ID STREQUAL MCXA145 OR CONFIG_DEVICE_ID STREQUAL MCXA146 OR CONFIG_DEVICE_ID STREQUAL MCXA152 OR CONFIG_DEVICE_ID STREQUAL MCXA153 OR CONFIG_DEVICE_ID STREQUAL MCXA154 OR CONFIG_DEVICE_ID STREQUAL MCXA155 OR CONFIG_DEVICE_ID STREQUAL MCXA156))

add_config_file(${CMAKE_CURRENT_LIST_DIR}/./output/npw/device_config/khci/usb_device_config.h ${CMAKE_CURRENT_LIST_DIR}/./output/npw/device_config/khci middleware_usb_device_khci_config_header)

else()

message(SEND_ERROR "middleware_usb_device_khci_config_header dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_device_ehci_config_header)
# Add set(CONFIG_USE_middleware_usb_device_ehci_config_header true) in config.cmake to use this component

message("middleware_usb_device_ehci_config_header component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if((CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1171xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1172xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1173xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1175xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1176xxxxx OR CONFIG_DEVICE_ID STREQUAL RW610 OR CONFIG_DEVICE_ID STREQUAL RW612 OR CONFIG_DEVICE_ID STREQUAL MIMXRT1181xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1182xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1187xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1189xxxxx OR CONFIG_DEVICE_ID STREQUAL MCXN546 OR CONFIG_DEVICE_ID STREQUAL MCXN547 OR CONFIG_DEVICE_ID STREQUAL MCXN946 OR CONFIG_DEVICE_ID STREQUAL MCXN947 OR CONFIG_DEVICE_ID STREQUAL MCXN235 OR CONFIG_DEVICE_ID STREQUAL MCXN236))

add_config_file(${CMAKE_CURRENT_LIST_DIR}/./output/npw/device_config/ehci/usb_device_config.h ${CMAKE_CURRENT_LIST_DIR}/./output/npw/device_config/ehci middleware_usb_device_ehci_config_header)

else()

message(SEND_ERROR "middleware_usb_device_ehci_config_header dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_common_header)
# Add set(CONFIG_USE_middleware_usb_common_header true) in config.cmake to use this component

message("middleware_usb_common_header component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_component_osa)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./include
)

else()

message(SEND_ERROR "middleware_usb_common_header dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_device_common_header)
# Add set(CONFIG_USE_middleware_usb_device_common_header true) in config.cmake to use this component

message("middleware_usb_device_common_header component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_component_osa AND CONFIG_USE_middleware_usb_common_header)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./device
)

else()

message(SEND_ERROR "middleware_usb_device_common_header dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_device_khci)
# Add set(CONFIG_USE_middleware_usb_device_khci true) in config.cmake to use this component

message("middleware_usb_device_khci component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if((CONFIG_DEVICE_ID STREQUAL MCXC143 OR CONFIG_DEVICE_ID STREQUAL MCXC144 OR CONFIG_DEVICE_ID STREQUAL MCXC243 OR CONFIG_DEVICE_ID STREQUAL MCXC244 OR CONFIG_DEVICE_ID STREQUAL MCXC443 OR CONFIG_DEVICE_ID STREQUAL MCXC444 OR CONFIG_DEVICE_ID STREQUAL MCXC141 OR CONFIG_DEVICE_ID STREQUAL MCXC142 OR CONFIG_DEVICE_ID STREQUAL MCXC242 OR CONFIG_DEVICE_ID STREQUAL MCXN546 OR CONFIG_DEVICE_ID STREQUAL MCXN547 OR CONFIG_DEVICE_ID STREQUAL MCXN946 OR CONFIG_DEVICE_ID STREQUAL MCXN947 OR CONFIG_DEVICE_ID STREQUAL MCXA132 OR CONFIG_DEVICE_ID STREQUAL MCXA133 OR CONFIG_DEVICE_ID STREQUAL MCXA142 OR CONFIG_DEVICE_ID STREQUAL MCXA143 OR CONFIG_DEVICE_ID STREQUAL MCXA144 OR CONFIG_DEVICE_ID STREQUAL MCXA145 OR CONFIG_DEVICE_ID STREQUAL MCXA146 OR CONFIG_DEVICE_ID STREQUAL MCXA152 OR CONFIG_DEVICE_ID STREQUAL MCXA153 OR CONFIG_DEVICE_ID STREQUAL MCXA154 OR CONFIG_DEVICE_ID STREQUAL MCXA155 OR CONFIG_DEVICE_ID STREQUAL MCXA156) AND CONFIG_USE_middleware_usb_device_khci_config_header AND CONFIG_USE_middleware_usb_device_common_header)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./device/usb_device_khci.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./device
  ${CMAKE_CURRENT_LIST_DIR}/./include
)

else()

message(SEND_ERROR "middleware_usb_device_khci dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_device_ehci)
# Add set(CONFIG_USE_middleware_usb_device_ehci true) in config.cmake to use this component

message("middleware_usb_device_ehci component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(((CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1171xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1172xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1173xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1175xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1176xxxxx OR CONFIG_DEVICE_ID STREQUAL RW610 OR CONFIG_DEVICE_ID STREQUAL RW612 OR CONFIG_DEVICE_ID STREQUAL MIMXRT1181xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1182xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1187xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1189xxxxx OR CONFIG_DEVICE_ID STREQUAL MCXN546 OR CONFIG_DEVICE_ID STREQUAL MCXN547 OR CONFIG_DEVICE_ID STREQUAL MCXN946 OR CONFIG_DEVICE_ID STREQUAL MCXN947 OR CONFIG_DEVICE_ID STREQUAL MCXN235 OR CONFIG_DEVICE_ID STREQUAL MCXN236) AND CONFIG_USE_middleware_usb_device_ehci_config_header AND CONFIG_USE_middleware_usb_device_common_header AND ((CONFIG_USE_middleware_usb_phy AND (CONFIG_DEVICE_ID STREQUAL MCXN235 OR CONFIG_DEVICE_ID STREQUAL MCXN236 OR CONFIG_DEVICE_ID STREQUAL MCXN546 OR CONFIG_DEVICE_ID STREQUAL MCXN547 OR CONFIG_DEVICE_ID STREQUAL MCXN946 OR CONFIG_DEVICE_ID STREQUAL MCXN947 OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1171xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1172xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1173xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1175xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1176xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1187xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1189xxxxx)) OR (NOT (CONFIG_NOT STREQUAL MCXN235 OR CONFIG_NOT STREQUAL MCXN236 OR CONFIG_NOT STREQUAL MCXN546 OR CONFIG_NOT STREQUAL MCXN547 OR CONFIG_NOT STREQUAL MCXN946 OR CONFIG_NOT STREQUAL MCXN947 OR CONFIG_NOT STREQUAL MIMXRT1061xxxxA OR CONFIG_NOT STREQUAL MIMXRT1061xxxxB OR CONFIG_NOT STREQUAL MIMXRT1062xxxxA OR CONFIG_NOT STREQUAL MIMXRT1062xxxxB OR CONFIG_NOT STREQUAL MIMXRT1171xxxxx OR CONFIG_NOT STREQUAL MIMXRT1172xxxxx OR CONFIG_NOT STREQUAL MIMXRT1173xxxxx OR CONFIG_NOT STREQUAL MIMXRT1175xxxxx OR CONFIG_NOT STREQUAL MIMXRT1176xxxxx OR CONFIG_NOT STREQUAL MIMXRT1187xxxxx OR CONFIG_NOT STREQUAL MIMXRT1189xxxxx))) AND (NOT (CONFIG_NOT STREQUAL RW612 OR CONFIG_NOT STREQUAL RW610))) OR ((CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1171xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1172xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1173xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1175xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1176xxxxx OR CONFIG_DEVICE_ID STREQUAL RW610 OR CONFIG_DEVICE_ID STREQUAL RW612 OR CONFIG_DEVICE_ID STREQUAL MIMXRT1181xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1182xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1187xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1189xxxxx OR CONFIG_DEVICE_ID STREQUAL MCXN546 OR CONFIG_DEVICE_ID STREQUAL MCXN547 OR CONFIG_DEVICE_ID STREQUAL MCXN946 OR CONFIG_DEVICE_ID STREQUAL MCXN947 OR CONFIG_DEVICE_ID STREQUAL MCXN235 OR CONFIG_DEVICE_ID STREQUAL MCXN236) AND CONFIG_USE_middleware_usb_device_ehci_config_header AND CONFIG_USE_middleware_usb_device_common_header AND CONFIG_USE_driver_memory AND (CONFIG_DEVICE_ID STREQUAL RW612 OR CONFIG_DEVICE_ID STREQUAL RW610)))

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./device/usb_device_ehci.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./device
  ${CMAKE_CURRENT_LIST_DIR}/./include
)

else()

message(SEND_ERROR "middleware_usb_device_ehci dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_phy)
# Add set(CONFIG_USE_middleware_usb_phy true) in config.cmake to use this component

message("middleware_usb_phy component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if((CONFIG_DEVICE_ID STREQUAL MCXN235 OR CONFIG_DEVICE_ID STREQUAL MCXN236 OR CONFIG_DEVICE_ID STREQUAL MCXN546 OR CONFIG_DEVICE_ID STREQUAL MCXN547 OR CONFIG_DEVICE_ID STREQUAL MCXN946 OR CONFIG_DEVICE_ID STREQUAL MCXN947 OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1171xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1172xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1173xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1175xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1176xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1187xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1189xxxxx) AND CONFIG_USE_middleware_usb_common_header AND ((CONFIG_USE_driver_memory AND (CONFIG_DEVICE_ID STREQUAL MIMXRT1173xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1175xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1176xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1187xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1189xxxxx)) OR (NOT (CONFIG_NOT STREQUAL MIMXRT1173xxxxx OR CONFIG_NOT STREQUAL MIMXRT1175xxxxx OR CONFIG_NOT STREQUAL MIMXRT1176xxxxx OR CONFIG_NOT STREQUAL MIMXRT1187xxxxx OR CONFIG_NOT STREQUAL MIMXRT1189xxxxx))))

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./phy/usb_phy.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./phy
)

else()

message(SEND_ERROR "middleware_usb_phy dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_device_stack_external)
# Add set(CONFIG_USE_middleware_usb_device_stack_external true) in config.cmake to use this component

message("middleware_usb_device_stack_external component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_component_osa AND CONFIG_USE_middleware_usb_device_controller_driver)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class/usb_device_class.c
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/usb_device_ch9.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device
)

else()

message(SEND_ERROR "middleware_usb_device_stack_external dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_device_audio_external)
# Add set(CONFIG_USE_middleware_usb_device_audio_external true) in config.cmake to use this component

message("middleware_usb_device_audio_external component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_device_stack_external)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class/usb_device_audio.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class
)

else()

message(SEND_ERROR "middleware_usb_device_audio_external dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_device_cdc_external)
# Add set(CONFIG_USE_middleware_usb_device_cdc_external true) in config.cmake to use this component

message("middleware_usb_device_cdc_external component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_device_stack_external)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class/usb_device_cdc_acm.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class
)

else()

message(SEND_ERROR "middleware_usb_device_cdc_external dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_device_cdc_rndis_external)
# Add set(CONFIG_USE_middleware_usb_device_cdc_rndis_external true) in config.cmake to use this component

message("middleware_usb_device_cdc_rndis_external component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_device_stack_external AND CONFIG_USE_middleware_usb_device_cdc_external)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class/usb_device_cdc_rndis.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class
)

else()

message(SEND_ERROR "middleware_usb_device_cdc_rndis_external dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_device_hid_external)
# Add set(CONFIG_USE_middleware_usb_device_hid_external true) in config.cmake to use this component

message("middleware_usb_device_hid_external component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_device_stack_external)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class/usb_device_hid.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class
)

else()

message(SEND_ERROR "middleware_usb_device_hid_external dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_device_dfu_external)
# Add set(CONFIG_USE_middleware_usb_device_dfu_external true) in config.cmake to use this component

message("middleware_usb_device_dfu_external component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_device_stack_external)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class/usb_device_dfu.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class
)

else()

message(SEND_ERROR "middleware_usb_device_dfu_external dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_device_msd_external)
# Add set(CONFIG_USE_middleware_usb_device_msd_external true) in config.cmake to use this component

message("middleware_usb_device_msd_external component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_device_stack_external)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class/usb_device_msc.c
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class/usb_device_msc_ufi.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class
)

else()

message(SEND_ERROR "middleware_usb_device_msd_external dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_device_phdc_external)
# Add set(CONFIG_USE_middleware_usb_device_phdc_external true) in config.cmake to use this component

message("middleware_usb_device_phdc_external component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_device_stack_external)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class/usb_device_phdc.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class
)

else()

message(SEND_ERROR "middleware_usb_device_phdc_external dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_device_video_external)
# Add set(CONFIG_USE_middleware_usb_device_video_external true) in config.cmake to use this component

message("middleware_usb_device_video_external component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_device_stack_external)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class/usb_device_video.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class
)

else()

message(SEND_ERROR "middleware_usb_device_video_external dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_device_ccid_external)
# Add set(CONFIG_USE_middleware_usb_device_ccid_external true) in config.cmake to use this component

message("middleware_usb_device_ccid_external component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_device_stack_external)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class/usb_device_ccid.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class
)

else()

message(SEND_ERROR "middleware_usb_device_ccid_external dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_device_printer_external)
# Add set(CONFIG_USE_middleware_usb_device_printer_external true) in config.cmake to use this component

message("middleware_usb_device_printer_external component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_device_stack_external)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class/usb_device_printer.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./output/source/device/class
)

else()

message(SEND_ERROR "middleware_usb_device_printer_external dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_phydcd)
# Add set(CONFIG_USE_middleware_usb_phydcd true) in config.cmake to use this component

message("middleware_usb_phydcd component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_phydcd_config_header AND (CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxB))

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./dcd/usb_phydcd.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./dcd
)

else()

message(SEND_ERROR "middleware_usb_phydcd dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_hsdcd)
# Add set(CONFIG_USE_middleware_usb_hsdcd true) in config.cmake to use this component

message("middleware_usb_hsdcd component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_hsdcd_config_header AND (CONFIG_DEVICE_ID STREQUAL MCXN236 OR CONFIG_DEVICE_ID STREQUAL MCXN546 OR CONFIG_DEVICE_ID STREQUAL MCXN547 OR CONFIG_DEVICE_ID STREQUAL MCXN946 OR CONFIG_DEVICE_ID STREQUAL MCXN947 OR CONFIG_DEVICE_ID STREQUAL MIMXRT1171xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1172xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1173xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1175xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1176xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1187xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1189xxxxx))

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./dcd/usb_hsdcd.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./dcd
)

else()

message(SEND_ERROR "middleware_usb_hsdcd dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_device_controller_driver)
# Add set(CONFIG_USE_middleware_usb_device_controller_driver true) in config.cmake to use this component

message("middleware_usb_device_controller_driver component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_component_osa AND (CONFIG_USE_middleware_usb_device_khci OR CONFIG_USE_middleware_usb_device_ehci))

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./device/usb_device_dci.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./device
)

else()

message(SEND_ERROR "middleware_usb_device_controller_driver dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_host_khci)
# Add set(CONFIG_USE_middleware_usb_host_khci true) in config.cmake to use this component

message("middleware_usb_host_khci component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if((CONFIG_DEVICE_ID STREQUAL MCXN546 OR CONFIG_DEVICE_ID STREQUAL MCXN547 OR CONFIG_DEVICE_ID STREQUAL MCXN946 OR CONFIG_DEVICE_ID STREQUAL MCXN947) AND CONFIG_USE_middleware_usb_host_khci_config_header AND CONFIG_USE_middleware_usb_host_common_header)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./host/usb_host_khci.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./host
  ${CMAKE_CURRENT_LIST_DIR}/./include
)

else()

message(SEND_ERROR "middleware_usb_host_khci dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_host_ehci)
# Add set(CONFIG_USE_middleware_usb_host_ehci true) in config.cmake to use this component

message("middleware_usb_host_ehci component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(((CONFIG_DEVICE_ID STREQUAL MCXN546 OR CONFIG_DEVICE_ID STREQUAL MCXN547 OR CONFIG_DEVICE_ID STREQUAL MCXN946 OR CONFIG_DEVICE_ID STREQUAL MCXN947 OR CONFIG_DEVICE_ID STREQUAL MCXN235 OR CONFIG_DEVICE_ID STREQUAL MCXN236 OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1171xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1172xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1173xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1175xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1176xxxxx OR CONFIG_DEVICE_ID STREQUAL RW610 OR CONFIG_DEVICE_ID STREQUAL RW612 OR CONFIG_DEVICE_ID STREQUAL MIMXRT1181xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1182xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1187xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1189xxxxx) AND CONFIG_USE_middleware_usb_host_ehci_config_header AND CONFIG_USE_middleware_usb_host_common_header AND ((CONFIG_USE_middleware_usb_phy AND (CONFIG_DEVICE_ID STREQUAL MCXN235 OR CONFIG_DEVICE_ID STREQUAL MCXN236 OR CONFIG_DEVICE_ID STREQUAL MCXN546 OR CONFIG_DEVICE_ID STREQUAL MCXN547 OR CONFIG_DEVICE_ID STREQUAL MCXN946 OR CONFIG_DEVICE_ID STREQUAL MCXN947 OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1171xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1172xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1173xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1175xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1176xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1187xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1189xxxxx)) OR (NOT (CONFIG_NOT STREQUAL MCXN235 OR CONFIG_NOT STREQUAL MCXN236 OR CONFIG_NOT STREQUAL MCXN546 OR CONFIG_NOT STREQUAL MCXN547 OR CONFIG_NOT STREQUAL MCXN946 OR CONFIG_NOT STREQUAL MCXN947 OR CONFIG_NOT STREQUAL MIMXRT1061xxxxA OR CONFIG_NOT STREQUAL MIMXRT1061xxxxB OR CONFIG_NOT STREQUAL MIMXRT1062xxxxA OR CONFIG_NOT STREQUAL MIMXRT1062xxxxB OR CONFIG_NOT STREQUAL MIMXRT1171xxxxx OR CONFIG_NOT STREQUAL MIMXRT1172xxxxx OR CONFIG_NOT STREQUAL MIMXRT1173xxxxx OR CONFIG_NOT STREQUAL MIMXRT1175xxxxx OR CONFIG_NOT STREQUAL MIMXRT1176xxxxx OR CONFIG_NOT STREQUAL MIMXRT1187xxxxx OR CONFIG_NOT STREQUAL MIMXRT1189xxxxx))) AND (NOT (CONFIG_NOT STREQUAL RW612 OR CONFIG_NOT STREQUAL RW610))) OR ((CONFIG_DEVICE_ID STREQUAL MCXN546 OR CONFIG_DEVICE_ID STREQUAL MCXN547 OR CONFIG_DEVICE_ID STREQUAL MCXN946 OR CONFIG_DEVICE_ID STREQUAL MCXN947 OR CONFIG_DEVICE_ID STREQUAL MCXN235 OR CONFIG_DEVICE_ID STREQUAL MCXN236 OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1171xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1172xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1173xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1175xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1176xxxxx OR CONFIG_DEVICE_ID STREQUAL RW610 OR CONFIG_DEVICE_ID STREQUAL RW612 OR CONFIG_DEVICE_ID STREQUAL MIMXRT1181xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1182xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1187xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1189xxxxx) AND CONFIG_USE_middleware_usb_host_ehci_config_header AND CONFIG_USE_middleware_usb_host_common_header AND CONFIG_USE_driver_memory AND (CONFIG_DEVICE_ID STREQUAL RW612 OR CONFIG_DEVICE_ID STREQUAL RW610)))

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./host/usb_host_ehci.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./host
  ${CMAKE_CURRENT_LIST_DIR}/./include
)

else()

message(SEND_ERROR "middleware_usb_host_ehci dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_host_audio)
# Add set(CONFIG_USE_middleware_usb_host_audio true) in config.cmake to use this component

message("middleware_usb_host_audio component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_host_stack)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./host/class/usb_host_audio.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./host/class
)

else()

message(SEND_ERROR "middleware_usb_host_audio dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_host_cdc)
# Add set(CONFIG_USE_middleware_usb_host_cdc true) in config.cmake to use this component

message("middleware_usb_host_cdc component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_host_stack)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./host/class/usb_host_cdc.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./host/class
)

else()

message(SEND_ERROR "middleware_usb_host_cdc dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_host_cdc_ecm)
# Add set(CONFIG_USE_middleware_usb_host_cdc_ecm true) in config.cmake to use this component

message("middleware_usb_host_cdc_ecm component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_host_stack AND CONFIG_USE_middleware_usb_host_cdc)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./host/class/usb_host_cdc_ecm.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./host/class
)

else()

message(SEND_ERROR "middleware_usb_host_cdc_ecm dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_host_cdc_rndis)
# Add set(CONFIG_USE_middleware_usb_host_cdc_rndis true) in config.cmake to use this component

message("middleware_usb_host_cdc_rndis component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_host_stack AND CONFIG_USE_middleware_usb_host_cdc)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./host/class/usb_host_cdc_rndis.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./host/class
)

else()

message(SEND_ERROR "middleware_usb_host_cdc_rndis dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_host_hid)
# Add set(CONFIG_USE_middleware_usb_host_hid true) in config.cmake to use this component

message("middleware_usb_host_hid component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_host_stack)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./host/class/usb_host_hid.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./host/class
)

else()

message(SEND_ERROR "middleware_usb_host_hid dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_host_msd)
# Add set(CONFIG_USE_middleware_usb_host_msd true) in config.cmake to use this component

message("middleware_usb_host_msd component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_host_stack)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./host/class/usb_host_msd.c
  ${CMAKE_CURRENT_LIST_DIR}/./host/class/usb_host_msd_ufi.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./host/class
)

else()

message(SEND_ERROR "middleware_usb_host_msd dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_host_video)
# Add set(CONFIG_USE_middleware_usb_host_video true) in config.cmake to use this component

message("middleware_usb_host_video component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_host_stack)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./host/class/usb_host_video.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./host/class
)

else()

message(SEND_ERROR "middleware_usb_host_video dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_host_phdc)
# Add set(CONFIG_USE_middleware_usb_host_phdc true) in config.cmake to use this component

message("middleware_usb_host_phdc component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_host_stack)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./host/class/usb_host_phdc.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./host/class
)

else()

message(SEND_ERROR "middleware_usb_host_phdc dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_host_printer)
# Add set(CONFIG_USE_middleware_usb_host_printer true) in config.cmake to use this component

message("middleware_usb_host_printer component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_middleware_usb_host_stack)

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./host/class/usb_host_printer.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./host/class
)

else()

message(SEND_ERROR "middleware_usb_host_printer dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_host_common_header)
# Add set(CONFIG_USE_middleware_usb_host_common_header true) in config.cmake to use this component

message("middleware_usb_host_common_header component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_component_osa AND CONFIG_USE_middleware_usb_common_header)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./host
)

else()

message(SEND_ERROR "middleware_usb_host_common_header dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_host_khci_config_header)
# Add set(CONFIG_USE_middleware_usb_host_khci_config_header true) in config.cmake to use this component

message("middleware_usb_host_khci_config_header component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if((CONFIG_DEVICE_ID STREQUAL MCXN546 OR CONFIG_DEVICE_ID STREQUAL MCXN547 OR CONFIG_DEVICE_ID STREQUAL MCXN946 OR CONFIG_DEVICE_ID STREQUAL MCXN947))

add_config_file(${CMAKE_CURRENT_LIST_DIR}/./output/npw/host_config/khci/usb_host_config.h ${CMAKE_CURRENT_LIST_DIR}/./output/npw/host_config/khci middleware_usb_host_khci_config_header)

else()

message(SEND_ERROR "middleware_usb_host_khci_config_header dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_host_ehci_config_header)
# Add set(CONFIG_USE_middleware_usb_host_ehci_config_header true) in config.cmake to use this component

message("middleware_usb_host_ehci_config_header component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if((CONFIG_DEVICE_ID STREQUAL MCXN546 OR CONFIG_DEVICE_ID STREQUAL MCXN547 OR CONFIG_DEVICE_ID STREQUAL MCXN946 OR CONFIG_DEVICE_ID STREQUAL MCXN947 OR CONFIG_DEVICE_ID STREQUAL MCXN235 OR CONFIG_DEVICE_ID STREQUAL MCXN236 OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1061xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxA OR CONFIG_DEVICE_ID STREQUAL MIMXRT1062xxxxB OR CONFIG_DEVICE_ID STREQUAL MIMXRT1171xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1172xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1173xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1175xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1176xxxxx OR CONFIG_DEVICE_ID STREQUAL RW610 OR CONFIG_DEVICE_ID STREQUAL RW612 OR CONFIG_DEVICE_ID STREQUAL MIMXRT1181xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1182xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1187xxxxx OR CONFIG_DEVICE_ID STREQUAL MIMXRT1189xxxxx))

add_config_file(${CMAKE_CURRENT_LIST_DIR}/./output/npw/host_config/ehci/usb_host_config.h ${CMAKE_CURRENT_LIST_DIR}/./output/npw/host_config/ehci middleware_usb_host_ehci_config_header)

else()

message(SEND_ERROR "middleware_usb_host_ehci_config_header dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()


if (CONFIG_USE_middleware_usb_host_stack)
# Add set(CONFIG_USE_middleware_usb_host_stack true) in config.cmake to use this component

message("middleware_usb_host_stack component is included from ${CMAKE_CURRENT_LIST_FILE}.")

if(CONFIG_USE_component_osa AND (CONFIG_USE_middleware_usb_host_khci OR CONFIG_USE_middleware_usb_host_ehci))

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/./host/usb_host_hci.c
  ${CMAKE_CURRENT_LIST_DIR}/./host/usb_host_devices.c
  ${CMAKE_CURRENT_LIST_DIR}/./host/usb_host_framework.c
  ${CMAKE_CURRENT_LIST_DIR}/./host/class/usb_host_hub.c
  ${CMAKE_CURRENT_LIST_DIR}/./host/class/usb_host_hub_app.c
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/./host
  ${CMAKE_CURRENT_LIST_DIR}/./host/class
  ${CMAKE_CURRENT_LIST_DIR}/./include
)

else()

message(SEND_ERROR "middleware_usb_host_stack dependency does not meet, please check ${CMAKE_CURRENT_LIST_FILE}.")

endif()

endif()

