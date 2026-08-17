import FEATURES from "../features.js";
import {createHash} from 'node:crypto'

// Dev plugin: provide mock ELRS endpoints to the local Vite server.
//
// Use this when you need a hardware-free UI workflow or deterministic test
// data. Do not use it to validate behavior that depends on a real device or
// real network timing.
export function devMockPlugin() {
    const PAGE_LOAD_DELAY_MS = 450
    const MODULE_LOAD_DELAY_MS = 800
    let cwGetRequestCount = 0

    function sendJSON(res, obj, status = 200) {
        res.statusCode = status
        res.setHeader('Content-Type', 'application/json')
        res.end(JSON.stringify(obj))
    }

    function sendText(res, text, status = 200) {
        res.statusCode = status
        res.setHeader('Content-Type', 'text/plain')
        res.end(text)
    }

    function sendStatus(res, status = 204) {
        res.statusCode = status
        res.end()
    }

    function sendDelayed(delayMs, sendResponse) {
        setTimeout(sendResponse, delayMs)
    }

    const hasLowBand = FEATURES.HAS_DUAL_BAND || FEATURES.HAS_SX127X
    const hasHighBand = FEATURES.HAS_DUAL_BAND || FEATURES.HAS_SX128X

    // Basic stub data used by multiple endpoints
    const stubState = {
        settings: {
            product_name: 'ELRS Mock Device',
            lua_name: "ELRS+PWM 2400RX",
            uidtype: 'Flashed',
            ssid: 'ExpressLRS TX',
            mode: 'AP',
            wifi_dbm: -60,
            custom_hardware: true,
            has_low_band: hasLowBand,
            has_high_band: hasHighBand,
            reg_domain_low: 'EU868',
            reg_domain_high: 'CE_LBT',
            target: "Unified_ESP32_LR1121",
            version: "25.0.0",
            "git-commit": "3468759",
            "module-type": FEATURES.IS_TX ? "TX" : "RX",
            "radio-type": FEATURES.HAS_SX128X ? "SX128X" : (FEATURES.HAS_LR1121 ? "LR1121" : (FEATURES.HAS_LR2021 ? "LR2021" : "SX127X")),
        },
        gyro: {
            version: 1.18,
            config_version: 8,
            imu: "LSM6Dxx"
        },
        options: {
            customised: true,
            "uid": [1, 2, 3, 4, 5, 6],   // this is the 'flashed' UID and may be empty if using traditional binding on an RX.
            "tlm-interval": 240,
            "fan-runtime": 30,
            "is-airport": true,
            "rcvr-uart-baud": 420000,
            "airport-uart-baud": 9600,
            "lock-on-first-connection": true,
            "domain": 1,
            "wifi-on-interval": 60,
            "wifi-password": "w1f1-pAssw0rd",
            "wifi-ssid": "network-ssid"
        },
        config: {
            uid: [5, 4, 3, 2, 1, 0],  // current UID, different to options if traditional binding or on-loan
            // RX config
            modelid: 62,
            'force-tlm': true,
            'serial-protocol': 1,
            'serial1-protocol': 0,
            'sbus-failsafe': 0,
            "pwm": [
                {"config": 0, "pin": 0, "features": 12},
                {"config": 1536, "pin": 4, "features": 12 + 16},
                {"config": 2048, "pin": 5, "features": 12 + 16},
                {"config": 3584, "pin": 1, "features": 1 + 4 + 8 + 16 + 32+ 64},
                {"config": 4608, "pin": 3, "features": 2 + 4 + 8 + 16 + 32+ 64}
            ],
            vbind: 0,
            // TX config
            "button-actions": [
                {
                    "color": 255,
                    "action": [
                        {"is-long-press": false, "count": 3, "action": 6},
                        {"is-long-press": true, "count": 5, "action": 1}
                    ]
                },
                {
                    "color": 224,
                    "action": [
                        {"is-long-press": false, "count": 2, "action": 3},
                        {"is-long-press": true, "count": 0, "action": 4}
                    ]
                }
            ]
        }
    }
    const GYRO_FUNCTION_NAMES = ['None', 'Aileron', 'Elevator', 'Rudder', 'Elevon', 'Elevon R', 'V-Tail', 'V-Tail R', 'Gyro Mode', 'Gyro Gain']
    const createGyroChannelFunction = (channel, overrides = {}) => {
        const channelFunction = {
            channel,
            functionId: 0,
            master: false,
            invert: false,
            min: 885,
            mid: 1500,
            max: 2135,
            ...overrides,
        }
        return {...channelFunction, function: GYRO_FUNCTION_NAMES[channelFunction.functionId] ?? 'None'}
    }
    const defaultGyroModes = () => [
        {modeId: 1, useRate: true, stickPriority: 0, gainFactor: 1, pitchLimit: 0, rollLimit: 0, trimPitch: 0, trimRoll: 0, gainPitch: 40, gainRoll: 30, gainYaw: 50},
        {modeId: 2, useRate: true, stickPriority: 0, gainFactor: 1, pitchLimit: 40, rollLimit: 70, trimPitch: 0, trimRoll: 0, gainPitch: 35, gainRoll: 35, gainYaw: 35},
        {modeId: 3, useRate: true, stickPriority: 0, gainFactor: 1, pitchLimit: 40, rollLimit: 70, trimPitch: 0, trimRoll: 0, gainPitch: 35, gainRoll: 35, gainYaw: 35},
        {modeId: 4, useRate: true, stickPriority: 0, gainFactor: 1, pitchLimit: 0, rollLimit: 0, trimPitch: 10, trimRoll: 0, gainPitch: 35, gainRoll: 35, gainYaw: 35},
        {modeId: 5, useRate: true, stickPriority: 0, gainFactor: 1, pitchLimit: 0, rollLimit: 0, trimPitch: 0, trimRoll: 0, gainPitch: 35, gainRoll: 35, gainYaw: 35},
    ]
    const gyroModes = defaultGyroModes()
    const defaultGyroPids = () => [
        {groupId: 0, axisId: 0, p: 35, i: 0, d: 10},
        {groupId: 0, axisId: 1, p: 35, i: 0, d: 10},
        {groupId: 0, axisId: 2, p: 35, i: 0, d: 10},
        {groupId: 1, axisId: 0, p: 35, i: 0, d: 10},
        {groupId: 1, axisId: 1, p: 35, i: 0, d: 10},
        {groupId: 1, axisId: 2, p: 35, i: 0, d: 10},
        {groupId: 2, axisId: 0, p: 20, i: 0, d: 0},
    ]
    const gyroPids = defaultGyroPids()

    const gyro = {
        version: 1.18,
        config_version: 9,
        enabled: false,
        imu: "LSM6Dxx",
        status: 2,
        status_bits: 0,
        next_action: "Run Orientation Wizard",
        error: "",
        link: 0,
        rate: 28,
        "read-errors": 0,
        "int-errors": 0,
        angle_r: 0,
        angle_p: 0,
        angle_y: 0,
        mode: 1,
        mode_position: 1,
        mode_switch_positions: 3,
        mode_map: [0, 1, 3],
        channels: Array.from({length: 16}, () => 1500),
        channel_functions: Array.from({length: 16}, (_unused, index) => createGyroChannelFunction(index + 1)),
    }
    gyro.stick_limits = gyro.channel_functions.map(() => ({min: 1500, mid: 1500, max: 1500}))
    gyro.channel_functions[0] = createGyroChannelFunction(1, {functionId: 1, master: true})
    gyro.channel_functions[1] = createGyroChannelFunction(2, {functionId: 2, master: true})
    gyro.channel_functions[3] = createGyroChannelFunction(4, {functionId: 3, master: true})
    gyro.channel_functions[8] = createGyroChannelFunction(9, {functionId: 8})
    gyro.channel_functions[9] = createGyroChannelFunction(10, {functionId: 9})
    const quickSetupGyro = ({wing, tail}) => {
        gyro.mode_switch_positions = 3
        gyro.mode_map = [0, 1, 3]
        gyro.channel_functions = Array.from({length: 16}, (_unused, index) => createGyroChannelFunction(index + 1))
        gyro.channel_functions[8] = createGyroChannelFunction(9, {functionId: 8})
        gyro.channel_functions[9] = createGyroChannelFunction(10, {functionId: 9})
        if (wing === 2) gyro.channel_functions[5] = createGyroChannelFunction(6, {functionId: 1})
        if (wing === 1 || wing === 2) gyro.channel_functions[0] = createGyroChannelFunction(1, {functionId: 1, master: true})
        if (wing === 3) {
            gyro.channel_functions[0] = createGyroChannelFunction(1, {functionId: 4, master: true})
            gyro.channel_functions[1] = createGyroChannelFunction(2, {functionId: 5})
        }
        if (tail === 1) {
            gyro.channel_functions[1] = createGyroChannelFunction(2, {functionId: 2, master: true})
            gyro.channel_functions[3] = createGyroChannelFunction(4, {functionId: 3, master: true})
        }
        if (tail === 2) {
            gyro.channel_functions[1] = createGyroChannelFunction(2, {functionId: 6, master: true})
            gyro.channel_functions[3] = createGyroChannelFunction(4, {functionId: 7})
        }
        if (tail === 3) {
            gyro.channel_functions[0] = createGyroChannelFunction(1, {functionId: 4})
            gyro.channel_functions[1] = createGyroChannelFunction(2, {functionId: 5})
            gyro.channel_functions[3] = createGyroChannelFunction(4, {functionId: 3, master: true})
        }
        if (tail === 4) gyro.channel_functions[3] = createGyroChannelFunction(4, {functionId: 3, master: true})
        gyroModes.splice(0, gyroModes.length, ...defaultGyroModes())
        gyroPids.splice(0, gyroPids.length, ...defaultGyroPids())
    }
    let gyroCalibrationStep = 'idle'
    const gyroRuntimeState = () => ({
        status: gyro.status,
        status_bits: gyro.status_bits,
        next_action: gyro.next_action,
        error: gyro.error,
        link: gyro.link,
        rate: gyro.rate,
        "read-errors": gyro["read-errors"],
        "int-errors": gyro["int-errors"],
        angle_r: gyro.angle_r,
        angle_p: gyro.angle_p,
        angle_y: gyro.angle_y,
        mode: gyro.mode,
        mode_position: gyro.mode_position,
        channels: gyro.channels,
        ...(gyroCalibrationStep.startsWith('sticks') ? {stick_limits: gyro.stick_limits} : {}),
    })
    let networkQueryCount = 0
    const hardwareState = {
        serial_rx: 3,
        serial_tx: 1,
        radio_miso: 33,
        radio_mosi: 32,
        radio_sck: 25,
        radio_busy: 36,
        radio_dio1: 37,
        radio_nss: 27,
        radio_rst: 15,
        radio_busy_2: 39,
        radio_dio1_2: 34,
        radio_nss_2: 13,
        radio_rst_2: 21,
        radio_rfo_hf: true,
        power_apc2: 26,
        power_min: 0,
        power_high: 6,
        power_max: 6,
        power_default: 3,
        power_control: 3,
        power_values: [120, 120, 120, 120, 120, 120, 100],
        power_values2: [-17, -15, -12, -9, -5, 0, 7],
        power_values_dual: [-18, -18, -15, -10, -6, -2, 2],
        radio_rfsw_ctrl: [31, 0, 20, 24, 24, 2, 0, 1],
        led_rgb: 22,
        led_rgb_isgrb: true,
        ledidx_rgb_status: [0],
        ledidx_rgb_boot: [0],
        radio_dcdc: true,
        use_backpack: true,
        debug_backpack_baud: 460800,
        debug_backpack_rx: 18,
        debug_backpack_tx: 5,
        backpack_boot: 23,
        backpack_en: 19,
        misc_fan_en: 2,
        vbat: 36,
        vbat_offset: 0,
        vbat_scale: 230,
        vbat_atten: 3,
        vbat_noreading: -1,
        vbat_cal_min: 5000,
        vbat_cal_max: 16000,
        vsrc1: 39,
        vsrc1_offset: 0,
        vsrc1_scale: 180,
        vsrc1_atten: 1,
        vsrc1_noreading: 8,
        // vsrc1_cal_min: 3300,
        // vsrc1_cal_max: 12600
    }
    const voltageSources = []

    function countVoltageSourcesFromHardware(hardware) {
        let count = 0
        if (hardware.vbat !== undefined && hardware.vbat !== '' && Number(hardware.vbat) >= 0) count++
        if (hardware.vsrc1 !== undefined && hardware.vsrc1 !== '' && Number(hardware.vsrc1) >= 0) count++
        if (hardware.vsrc2 !== undefined && hardware.vsrc2 !== '' && Number(hardware.vsrc2) >= 0) count++
        if (hardware.vsrc3 !== undefined && hardware.vsrc3 !== '' && Number(hardware.vsrc3) >= 0) count++
        return count
    }

    function syncVoltageSourcesFromHardware(hardware) {
        const sourceDefs = [
            {id: 'vbat', label: 'VBat'},
            {id: 'vsrc1', label: 'VSrc1'},
            {id: 'vsrc2', label: 'VSrc2'},
            {id: 'vsrc3', label: 'VSrc3'},
        ]
        voltageSources.length = 0
        for (const sourceDef of sourceDefs) {
            const pin = hardware[sourceDef.id]
            if (pin === undefined || pin === '' || Number(pin) < 0) continue
            voltageSources.push({
                id: sourceDef.id,
                label: sourceDef.label,
                pin: Number(pin),
                defined: true,
                offset: Number(hardware[`${sourceDef.id}_offset`] ?? 0),
                scale: Number(hardware[`${sourceDef.id}_scale`] ?? 200),
                atten: Number(hardware[`${sourceDef.id}_atten`] ?? -1),
                noReading: Number(hardware[`${sourceDef.id}_noreading`] ?? -1),
                calMin: Number(hardware[`${sourceDef.id}_cal_min`] ?? 0),
                calMax: Number(hardware[`${sourceDef.id}_cal_max`] ?? 0),
            })
        }
        if (stubState.settings['module-type'] === 'RX') {
            stubState.settings.voltage_source_count = countVoltageSourcesFromHardware(hardware)
        } else {
            delete stubState.settings.voltage_source_count
        }
    }

    syncVoltageSourcesFromHardware(hardwareState)

    function jitter(amount = 1) {
        return Math.round((Math.random() * amount * 2) - amount)
    }

    function mockVoltageSample(atten, base, span, readingFloor = 0) {
        const attenGain = atten <= 3 ? (4 - atten) : (8 - atten)
        const rawMedian = Math.max(readingFloor, Math.min(4095, Math.round(base + span * attenGain + jitter(2))))
        const spread = 2 + Math.abs(jitter(1))
        const rawMax = Math.min(4095, rawMedian + spread + Math.abs(jitter(1)))
        const adcMedian = Math.max(0, Math.min(4095, rawMedian + jitter(1)))
        return {
            rawMax,
            adcMedian,
            saturated: rawMax >= 4071,
            hasReading: rawMedian > readingFloor
        }
    }

    return {
        name: 'vite-dev-mock',
        apply: 'serve',
        configureServer(server) {
            server.httpServer?.on('upgrade', (req, socket) => {
                if ((req.url || '').split('?')[0] !== '/gyro-runtime') return

                const key = req.headers['sec-websocket-key']
                if (typeof key !== 'string') {
                    socket.destroy()
                    return
                }

                const accept = createHash('sha1')
                    .update(`${key}258EAFA5-E914-47DA-95CA-C5AB0DC85B11`)
                    .digest('base64')
                socket.write(`HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: ${accept}\r\n\r\n`)

                const sendRuntime = () => {
                    if (!socket.writable || socket.writableLength !== 0) return
                    const payload = Buffer.from(JSON.stringify(gyroRuntimeState()))
                    const header = payload.length < 126
                        ? Buffer.from([0x81, payload.length])
                        : Buffer.from([0x81, 126, payload.length >> 8, payload.length & 0xff])
                    socket.write(header)
                    socket.write(payload)
                }
                const publisher = setInterval(sendRuntime, 40)
                socket.once('close', () => clearInterval(publisher))
                socket.once('error', () => clearInterval(publisher))
                socket.on('data', (data) => {
                    if ((data[0] & 0x0f) === 0x08) socket.end()
                })
                sendRuntime()
            })
            server.middlewares.use((req, res, next) => {
                const url = req.url || '/'
                const method = (req.method || 'GET').toUpperCase()

                // Add delay for lazy-loaded page group modules to simulate network latency
                if (method === 'GET' && (url.includes('general-group.js') || url.includes('advanced-group.js'))) {
                    return setTimeout(() => next(), MODULE_LOAD_DELAY_MS)
                }

                // Utilities to collect request body (json or text)
                const readBody = () => new Promise((resolve) => {
                    let data = ''
                    req.on('data', (chunk) => {
                        data += chunk
                    })
                    req.on('end', () => resolve(data))
                })

                if (method === 'GET' && url === '/config?export') {
                    return sendJSON(res, stubState.config)
                }
                if (method === 'GET' && url === '/config') {
                    // Reset the networks scan delay counter whenever config is fetched
                    networkQueryCount = 0
                    if (stubState.settings['module-type'] === 'RX') {
                        stubState.settings.voltage_source_count = voltageSources.length
                    } else {
                        delete stubState.settings.voltage_source_count
                    }
                    return sendDelayed(PAGE_LOAD_DELAY_MS, () => sendJSON(res, stubState))
                }
                if (method === 'GET' && (url === '/networks.json' || url.startsWith('/networks.json'))) {
                    networkQueryCount++
                    if (networkQueryCount <= 3) {
                        return sendStatus(res, 204)
                    }
                    return sendJSON(res, ['ExpressLRS TX', 'MockHomeWiFi', 'OfficeNet'])
                }
                if (method === 'POST' && (url === '/options' || url === '/options.json')) {
                    return readBody().then(() => sendText(res, 'Options saved'))
                }
                if (method === 'POST' && url === '/config') {
                    return readBody().then((body) => {
                        try {
                            const data = JSON.parse(body || '{}')
                            const pwm = stubState.config.pwm
                            stubState.config = {...data, pwm}
                            if (data.pwm) {
                                let i = 0
                                stubState.config.pwm.forEach((item) => {
                                    item.config = data.pwm[i++]
                                })
                            }
                        } catch (e) {
                            // ignore parse errors in mock
                        }
                        return sendText(res, 'Config saved')
                    })
                }
                if (method === 'GET' && url === '/cw') {
                    cwGetRequestCount++
                    // Fail every third request to test error handling
                    if (cwGetRequestCount % 3 === 0) {
                        return sendDelayed(PAGE_LOAD_DELAY_MS, () => sendStatus(res, 500))
                    }
                    const cwConfig = {
                        radios: hardwareState.radio_nss_2 === undefined ? 1 : 2,
                        center: FEATURES.HAS_SX128X ? 2440000000 : 868000000
                    }
                    if (FEATURES.HAS_LR1121) {
                        cwConfig.center2 = 2400000000
                    }
                    return sendDelayed(PAGE_LOAD_DELAY_MS, () => sendJSON(res, cwConfig))
                }
                if (method === 'POST' && url === '/cw') {
                    return readBody().then(() => sendText(res, 'CW started'))
                }
                if (method === 'POST' && url === '/buttons') {
                    return readBody().then(() => sendText(res, 'ok'))
                }
                if (method === 'POST' && url === '/update') {
                    return readBody().then(() => sendJSON(res, {
                        status: 'ok',
                        msg: 'Firmware updated successfully. Device will reboot.'
                    }))
                }
                if (method === 'POST' && url === '/forceupdate') {
                    return readBody().then(() => sendJSON(res, {
                        status: 'ok',
                        msg: 'Forced firmware update started.'
                    }))
                }
                if (method === 'POST' && (url === '/reset?lr1121' || url === '/reset')) {
                    return sendText(res, 'LR1121 reset requested. Rebooting...')
                }
                if (method === 'GET' && url === '/lr1121.json') {
                    return sendDelayed(PAGE_LOAD_DELAY_MS, () => sendJSON(res, {
                        manual: false,
                        radio1: {type: 0x07, hardware: 0x11, firmware: 0x1234},
                        radio2: FEATURES.HAS_LR1121 ? {type: 0x07, hardware: 0x12, firmware: 0x1234} : undefined
                    }))
                }
                if (method === 'POST' && url === '/lr1121') {
                    return readBody().then(() => sendJSON(res, {status: 'ok', msg: 'LR1121 firmware updated'}))
                }
                if (method === 'GET' && url === '/hardware.json') {
                    return sendDelayed(PAGE_LOAD_DELAY_MS, () => sendJSON(res, hardwareState))
                }
                if (method === 'GET' && url === '/gyro.json') {
                    return sendJSON(res, gyroRuntimeState())
                }
                if (method === 'GET' && url.startsWith('/gyro-config.json')) {
                    return sendJSON(res, {
                        enabled: gyro.enabled,
                        imu: gyro.imu,
                        version: gyro.version,
                        config_version: gyro.config_version,
                        mode_switch_positions: gyro.mode_switch_positions,
                        mode_map: gyro.mode_map,
                        channel_functions: url.includes('?export')
                            ? gyro.channel_functions.filter((channelFunction) => channelFunction.functionId !== 0)
                            : gyro.channel_functions,
                        gyro_modes: gyroModes,
                        gyro_pids: gyroPids,
                    })
                }
                if (method === 'POST' && url === '/gyro-calibration.json') {
                    return readBody().then((body) => {
                        const action = JSON.parse(body || '{}').action
                        if (action === 'orientation-horizontal' && gyroCalibrationStep === 'idle') {
                            gyroCalibrationStep = 'orientation-horizontal'
                        } else if (action === 'orientation-vertical' && gyroCalibrationStep === 'orientation-horizontal') {
                            gyro.status_bits |= 1 << 1
                            gyroCalibrationStep = 'idle'
                        } else if (action === 'level' && gyroCalibrationStep === 'idle') {
                            gyro.status_bits |= 1 << 0
                        } else if (action === 'sticks-center' && gyroCalibrationStep === 'idle') {
                            gyroCalibrationStep = 'sticks-centered'
                        } else if (action === 'sticks-range-start' && (gyroCalibrationStep === 'idle' || gyroCalibrationStep === 'sticks-centered')) {
                            gyroCalibrationStep = 'sticks-range'
                            gyro.stick_limits = gyro.stick_limits.map((limits, index) => gyro.channel_functions[index].functionId
                                ? {...limits, min: 885, max: 2135}
                                : limits)
                        } else if (action === 'sticks-range-finish' && gyroCalibrationStep === 'sticks-range') {
                            gyro.status_bits |= 1 << 2
                            gyroCalibrationStep = 'idle'
                        } else if (action === 'cancel') {
                            gyroCalibrationStep = 'idle'
                        } else {
                            return sendText(res, 'Invalid gyro calibration action', 400)
                        }
                        return sendJSON(res, {status_bits: gyro.status_bits})
                    })
                }
                if (method === 'POST' && url.startsWith('/gyro-config.json')) {
                    return readBody().then((body) => {
                        let data
                        try {
                            data = JSON.parse(body || '{}')
                        } catch (_error) {
                            return sendText(res, 'Invalid JSON', 400)
                        }
                        if (url.includes('?import')
                            && (typeof data.enabled !== 'boolean'
                                || !Number.isInteger(data.mode_switch_positions)
                                || !Array.isArray(data.mode_map)
                                || !Array.isArray(data.channel_functions)
                                || !Array.isArray(data.gyro_modes)
                                || !Array.isArray(data.gyro_pids))) {
                            return sendText(res, 'Incomplete gyro configuration', 400)
                        }
                        if (data.quick_setup) {
                            quickSetupGyro({
                                wing: Number(data.quick_setup.wing),
                                tail: Number(data.quick_setup.tail),
                            })
                        } else {
                            if (Array.isArray(data.channel_functions)) {
                                const channelFunctions = Array.from({length: 16}, (_unused, index) => createGyroChannelFunction(index + 1))
                                for (const row of data.channel_functions) {
                                    const channel = Number(row?.channel)
                                    if (channel >= 1 && channel <= channelFunctions.length) {
                                        channelFunctions[channel - 1] = createGyroChannelFunction(channel, row)
                                    }
                                }
                                gyro.channel_functions = channelFunctions
                            }
                            if (data.mode_switch_positions !== undefined) {
                                const positions = Number(data.mode_switch_positions)
                                if (positions < 2 || positions > 6) {
                                    return sendText(res, 'Invalid mode switch position count', 400)
                                }
                                gyro.mode_switch_positions = positions
                            }
                            if (Array.isArray(data.mode_map)) {
                                gyro.mode_map = data.mode_map.slice(0, gyro.mode_switch_positions).map((value) => Number(value) || 0)
                            }
                            if (data.enabled !== undefined) {
                                gyro.enabled = Boolean(data.enabled)
                            }
                            if (Array.isArray(data.gyro_modes)) {
                                gyroModes.splice(0, gyroModes.length, ...data.gyro_modes.map((mode) => ({...mode, modeId: Number(mode.modeId)})))
                            }
                            if (Array.isArray(data.gyro_pids)) {
                                const nextPids = gyroPids.map((pid) => ({...pid}))
                                const seen = new Set()
                                for (const row of data.gyro_pids) {
                                    const groupId = Number(row?.groupId)
                                    const axisId = Number(row?.axisId)
                                    const key = `${groupId}:${axisId}`
                                    const target = nextPids.find((pid) => pid.groupId === groupId && pid.axisId === axisId)
                                    const pMin = groupId === 2 ? 10 : 0
                                    const pMax = groupId === 2 ? 60 : 100
                                    if (!target || seen.has(key)
                                        || !Number.isInteger(row.p) || row.p < pMin || row.p > pMax
                                        || !Number.isInteger(row.i) || row.i < 0 || row.i > 100
                                        || !Number.isInteger(row.d) || row.d < 0 || row.d > 100) {
                                        return sendText(res, 'Invalid PID configuration', 400)
                                    }
                                    seen.add(key)
                                    Object.assign(target, {p: row.p, i: row.i, d: row.d})
                                }
                                if (url.includes('?import') && seen.size !== nextPids.length) {
                                    return sendText(res, 'Incomplete PIDs', 400)
                                }
                                gyroPids.splice(0, gyroPids.length, ...nextPids)
                            }
                        }
                        return sendText(res, 'Gyro configuration updated')
                    })
                }
                if (method === 'POST' && url === '/hardware.json') {
                    return readBody().then((body) => {
                        try {
                            const data = JSON.parse(body || '{}')
                            Object.assign(hardwareState, data)
                            syncVoltageSourcesFromHardware(hardwareState)
                        } catch (e) {
                            // ignore parse errors in mock
                        }
                        return sendText(res, 'Hardware saved')
                    })
                }
                if (method === 'POST' && url === '/reboot') {
                    return sendText(res, 'Rebooting')
                }
                if (method === 'POST' && url === '/binding') {
                    return sendText(res, 'Binding')
                }
                if (method === 'POST' && url === '/sethome') {
                    return sendText(res, 'Home set')
                }
                if (method === 'POST' && url === '/import') {
                    return sendText(res, 'Import complete')
                }
                if (method === 'GET' && url === '/import') {
                    return sendText(res, JSON.stringify(stubState))
                }
                if (method === 'POST' && url === '/voltage-sample') {
                    return readBody().then((body) => {
                        let payload = {}
                        try {
                            payload = JSON.parse(body || '{}')
                        } catch (_e) {
                        }

                        const sampleForRequest = (requestPayload = {}) => {
                            const sourceId = requestPayload.source || 'vbat'
                            const source = voltageSources.find((item) => item.id === sourceId) || voltageSources[0]
                            if (!source) {
                                return {hasReading: false}
                            }

                            const stage = Number(requestPayload.stage ?? 0)
                            const atten = Number(requestPayload.atten ?? source.atten ?? 0)

                            let base = 180
                            let span = 180
                            let readingFloor = 0

                            if (stage === 0) {
                                base = 700
                                span = 320
                            } else if (stage === 1) {
                                base = 320
                                span = 150
                            } else if (stage === 2) {
                                base = Math.max(0, source.noReading - 12)
                                span = 4
                                readingFloor = Math.max(0, source.noReading - 24)
                            } else if (stage === 3) {
                                base = 540
                                span = 210
                            }

                            return mockVoltageSample(atten, base, span, readingFloor)
                        }

                        const requests = Array.isArray(payload.requests) ? payload.requests : []
                        const samples = {}
                        for (const requestPayload of requests) {
                            const sourceId = requestPayload?.source
                            if (!sourceId) continue
                            samples[sourceId] = sampleForRequest(requestPayload)
                        }
                        return sendJSON(res, {samples})
                    })
                }

                return next()
            })
        }
    }
}
