from types import SimpleNamespace
import re

import binary_configurator
import binary_flash
from binary_flash import UploadMethod


def _get_upload_method(target_name: str) -> UploadMethod:
    if "_BETAFLIGHTPASSTHROUGH" in target_name:
        return UploadMethod.betaflight
    if "_UART" in target_name:
        return UploadMethod.uart
    raise ValueError(f"Unsupported PIO upload target '{target_name}'")


def upload_via_pio(source, target, env):
    firmware_path = str(source[0])
    pio_target = target[0].name
    target_name = env['PIOENV'].upper()

    args = SimpleNamespace(
        target=None,
        firmware=re.sub('_VIA_.*', '', target_name),
        tx='_TX_' in target_name,
        rx_as_tx=None,
        file=open(firmware_path, 'r+b'),
        flash=_get_upload_method(target_name),
        erase=False,
        out=None,
        port=env.get('UPLOAD_PORT', None),
        baud=0,
        force=pio_target == 'uploadforce',
        confirm=False,
    )

    args.target, config = binary_configurator.ask_for_firmware(args)
    args.options_json = env['OPTIONS_JSON']
    with args.file as firmware_file:
        options = binary_configurator.configure_firmware_file(
            firmware_file,
            args,
            config,
            target_name,
            env.get('DEVICE_NAME', None)
        )

    binary_configurator.prepare_flash_args(args, config)
    return binary_flash.upload(options, args)
