"""Offline only: compiles and executes native C tests, builds ARM, never probes hardware."""
import argparse, hashlib, json, pathlib, re, subprocess, sys, os
ROOT = pathlib.Path(__file__).resolve().parent
os.chdir(ROOT)
ap = argparse.ArgumentParser()
ap.add_argument('--firmware', default=str(ROOT.parent / 'GEArm-H723-Firmware'))
ap.add_argument('--host-cc', default='F:/Tool/MATLAB R2025a(64bit)/toolbox/sldrt/clang/win64/clang.exe')
args = ap.parse_args()
fw = pathlib.Path(args.firmware).resolve()
out = ROOT / 'build'
out.mkdir(exist_ok=True)
log = (out / 'build.log').open('w', encoding='utf8')
result = {'host_tests': 'not_run', 'arm_build': 'not_run', 'hardware': 'not_connected', 'artifacts': {}}

def run(cmd, check=True):
    log.write('$ ' + subprocess.list2cmdline([str(x) for x in cmd]) + '\n')
    log.flush()
    p = subprocess.run([str(x) for x in cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors='replace')
    log.write(p.stdout)
    log.flush()
    if check and p.returncode:
        raise RuntimeError(f'exit {p.returncode}: {cmd[0]}\n{p.stdout}')
    return p.stdout, p.returncode

try:
    gcc = next(pathlib.Path('F:/MCU/CubeIDE').rglob('arm-none-eabi-gcc.exe'))
    bindir = gcc.parent
    result['compiler'] = run([gcc, '--version'])[0]
    result['host_compiler'] = run([args.host_cc, '--version'])[0]
    host = [args.host_cc, '--target=i686-w64-windows-gnu', '--sysroot=C:/MinGW', '-B', 'C:/MinGW/bin',
            '-ffreestanding', '-nostdlib', '-std=c11', '-Wall', '-Wextra', '-Werror', '-I.',
            'protocol.c', 'tests.c', 'host_entry.c', '-L', 'C:/MinGW/lib', '-lmsvcrt',
            '-Wl,-e,_mainCRTStartup', '-o', out / 'host_tests.exe']
    run(host)
    host_out, host_rc = run([out / 'host_tests.exe'], check=False)
    result['host_test_output'] = host_out
    m = re.search(r'HOST_TESTS pass=(\d+) fail=(\d+)', host_out)
    if not m:
        raise RuntimeError('host test summary missing\n' + host_out)
    result['host_pass'] = int(m.group(1))
    result['host_fail'] = int(m.group(2))
    if host_rc or result['host_fail']:
        raise RuntimeError(f"host tests failed pass={result['host_pass']} fail={result['host_fail']}")
    result['host_tests'] = 'passed'
    inc = ['.', fw / 'Core/Inc', fw / 'Drivers/STM32H7xx_HAL_Driver/Inc', fw / 'Drivers/CMSIS/Include',
           fw / 'Drivers/CMSIS/Device/ST/STM32H7xx/Include',
           ROOT / 'usb/hal', ROOT / 'usb/App', ROOT / 'usb/Target',
           ROOT / 'usb/Middlewares/ST/STM32_USB_Device_Library/Core/Inc',
           ROOT / 'usb/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc']
    flags = ['-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-d16', '-mfloat-abi=hard', '-DSTM32H723xx',
             '-DUSE_HAL_DRIVER', '-DHSE_VALUE=24000000UL', '-DUSE_USB_HS', '-DHAL_PCD_MODULE_ENABLED', '-DHAL_SPI_MODULE_ENABLED', '-std=c11', '-Os', '-g3',
             '-ffunction-sections', '-fdata-sections'] + ['-I' + str(p) for p in inc]
    hal = ['hal', 'hal_cortex', 'hal_rcc', 'hal_rcc_ex', 'hal_pwr', 'hal_pwr_ex', 'hal_flash',
           'hal_flash_ex', 'hal_gpio', 'hal_dma', 'hal_dma_ex', 'hal_uart', 'hal_uart_ex', 'hal_fdcan']
    sources = [ROOT / 'main.c', ROOT / 'protocol.c', ROOT / 'runtime.c', fw / 'Core/Src/system_stm32h7xx.c',
               fw / 'Core/Startup/startup_stm32h723vgtx.s']
    sources += [fw / 'Drivers/STM32H7xx_HAL_Driver/Src' / ('stm32h7xx_' + s + '.c') for s in hal]
    # USB CDC stack (copied from Damiao's official CtrBoard-H7_USB example)
    sources += [ROOT / 'usb/hal/stm32h7xx_hal_pcd.c',
                ROOT / 'usb/hal/stm32h7xx_hal_pcd_ex.c',
                ROOT / 'usb/hal/stm32h7xx_ll_usb.c',
                ROOT / 'usb/hal/stm32h7xx_hal_spi.c',
                ROOT / 'usb/hal/stm32h7xx_hal_spi_ex.c',
                ROOT / 'usb/App/usb_device.c',
                ROOT / 'usb/App/usbd_desc.c',
                ROOT / 'usb/App/usbd_cdc_if.c',
                ROOT / 'usb/Target/usbd_conf.c',
                ROOT / 'usb/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c',
                ROOT / 'usb/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c',
                ROOT / 'usb/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c',
                ROOT / 'usb/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c']
    objects = []
    references = {}
    for i, s in enumerate(sources):
        obj = out / (str(i) + '_' + s.stem + '.o')
        objects.append(obj)
        warnings = ['-Wall', '-Wextra', '-Werror'] if s.parent == ROOT else []
        run([gcc, *flags, *warnings, '-c', s, '-o', obj])
        if s.parent != ROOT:
            references[str(s)] = hashlib.sha256(s.read_bytes()).hexdigest()
    linker = fw / 'STM32H723VGTX_FLASH.ld'
    references[str(linker)] = hashlib.sha256(linker.read_bytes()).hexdigest()
    elf = out / 'mc02_serial_test.elf'
    run([gcc, *flags, *objects, '-nostdlib', '-T', linker,
         '-Wl,--gc-sections,--fatal-warnings,-Map=' + str(out / 'mc02_serial_test.map'), '-o', elf])
    for fmt, ext in [('ihex', 'hex'), ('binary', 'bin')]:
        run([bindir / 'arm-none-eabi-objcopy.exe', '-O', fmt, elf, out / ('mc02_serial_test.' + ext)])
    size = run([bindir / 'arm-none-eabi-size.exe', elf])[0]
    (out / 'size.txt').write_text(size, encoding='utf8')
    nm = run([bindir / 'arm-none-eabi-nm.exe', '-S', '-n', elf])[0]
    (out / 'symbols.txt').write_text(nm, encoding='utf8')
    wanted = {'link', 'USART1_IRQHandler', 'SysTick_Handler', 'FDCAN1_IT0_IRQHandler', 'FDCAN1_IT1_IRQHandler'}
    result['swd_symbols'] = [line for line in nm.splitlines() if line.split()[-1] in wanted]
    result['size'] = size
    result['references_sha256'] = references
    result['arm_build'] = 'passed'
    result['source_sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in ROOT.iterdir() if p.suffix in ['.c', '.h', '.py']}
    result['tested'] = ['native shared protocol/state machine with fake transport/time', 'real ARM GCC compile/link', 'nm/size/artifact hashes', 'pc_service.py --selftest is separate and does not open a port']
    result['untested'] = ['真实启动', 'HSE 锁定', 'USB CDC 枚举与 COM 号分配', 'CAN 真实回包', 'VOS/Flash 时序', 'TX 取消时序', 'WS2812/SPI6 时序（已从关键路径移除）']
    for name in ['mc02_serial_test.elf', 'mc02_serial_test.hex', 'mc02_serial_test.bin', 'mc02_serial_test.map', 'host_tests.exe', 'symbols.txt', 'size.txt']:
        p = out / name
        result['artifacts'][name] = {'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest(), 'bytes': p.stat().st_size}
except Exception as e:
    result['error'] = str(e)
    print(e, file=sys.stderr)
finally:
    log.close()
    (ROOT / 'result.json').write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding='utf8')
if 'error' in result:
    sys.exit(1)
print(json.dumps({k: result[k] for k in ['host_tests', 'host_pass', 'host_fail', 'arm_build', 'swd_symbols']}, indent=2, ensure_ascii=False))