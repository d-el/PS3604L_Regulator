INCLUDE memory.ld

MEMORY
{
    FLASH (rx)      : ORIGIN = _main_flash_start, LENGTH = _main_flash_size
    FLASHS (r)      : ORIGIN = _settings_flash_start, LENGTH = _settings_flash_size
    RAM (xrw)       : ORIGIN = _ram_start, LENGTH = _ram_size
}

INCLUDE infosection.ld
INCLUDE common_cortexm.ld

SECTIONS
{
  /* User settings */
  ._user_settings (NOLOAD):
  {
    _suser_settings = .;
    . = . + _settings_flash_size;
    _euser_settings = .;
  } >FLASHS
}
