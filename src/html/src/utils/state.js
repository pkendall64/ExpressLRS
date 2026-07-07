import {State} from "@lit-app/state"
import {loadJSON, postJSON, saveJSONWithReboot, showAlert} from "./feedback.js"

class ElrsState extends State {
    config = {}
    options = {}
    settings = {}
    hardware = {}
}

export function setHardwareState(hardware) {
    elrsState.hardware = {...hardware}
    elrsState.settings = {
        ...elrsState.settings,
        custom_hardware: !!hardware.customised
    }
}
export function applyStatePatch(data) {
    if (!data || typeof data !== 'object') return
    if (data.settings) {
        elrsState.settings = {...elrsState.settings, ...data.settings}
    }
    if (data.options) {
        elrsState.options = {...data.options}
    }
    if (data.config) {
        elrsState.config = {...data.config}
    }
    if (data.hardware) {
        setHardwareState(data.hardware)
    }
}


export async function loadHardware(force = false) {
    if (!force && Object.keys(elrsState.hardware || {}).length > 0) {
        return elrsState.hardware
    }

    const hardware = await loadJSON('/hardware.json')
    setHardwareState(hardware)
    return hardware
}

export function formatBand() {
    if (elrsState.settings) {
        if (elrsState.settings.has_low_band && elrsState.settings.has_high_band) {
            return elrsState.settings.reg_domain_low + '/' + elrsState.settings.reg_domain_high
        }
        if (elrsState.settings.has_low_band)
            return elrsState.settings.reg_domain_low
        return elrsState.settings.reg_domain_high
    }
}

export function formatWifiRssi() {
    let wifiDesc = ""
    if (elrsState.settings) {
        if (elrsState.settings.mode === "STA")
            wifiDesc = `Client [${elrsState.settings.ssid}]`
        else
            wifiDesc = "AP mode"

        // If we have dbm and it isn't 0, add it
        if (elrsState.settings?.wifi_dbm)
        {
            let dbm = elrsState.settings.wifi_dbm
            wifiDesc += ` ${dbm}dBm `
            if (dbm > -30)
                wifiDesc += "**GODLIKE**"
            else if (dbm > -60)
                wifiDesc += "excellent signal"
            else if (dbm == -69)
                wifiDesc += "nice signal"
            else if (dbm > -80)
                wifiDesc += "good signal"
            else if (dbm > -90)
                wifiDesc += "poor signal"
            else
                wifiDesc += "Unusable"
        }
    }
    return wifiDesc
}

export function saveConfig(changes, successCB) {
    const currentPWM = elrsState.config.pwm
    if (changes.pwm) {
        for (let i = 0; i < currentPWM.length; i++) {
            currentPWM[i].config = changes.pwm[i]
        }
    } else if (currentPWM) {
        changes.pwm = []
        for (let i = 0; i < currentPWM.length; i++) {
            changes.pwm.push(currentPWM[i].config)
        }
    }
    const newConfig = {...elrsState.config, ...changes}
    return saveJSONWithReboot('Configuration Update Succeeded', 'Configuration Update Failed', '/config', newConfig, (response) => {
        applyStatePatch(response)
        if (successCB) successCB(response)
    })
}

export function saveOptions(changes, successCB) {
    const newOptions = {...elrsState.options, ...changes, customised: true}
    return saveJSONWithReboot('Configuration Update Succeeded', 'Configuration Update Failed', '/options.json', newOptions, (response) => {
        applyStatePatch(response)
        if (successCB) successCB(response)
    })
}

export async function saveOptionsAndConfig(changes, successCB) {
    const newOptions = {...elrsState.options, ...changes.options, customised: true}
    try {
        const optionsResponse = await postJSON('/options.json', newOptions)
        applyStatePatch(optionsResponse)
        await saveConfig(changes.config, (configResponse) => {
            if (successCB) successCB(configResponse)
        })
    } catch (error) {
        await showAlert('error', 'Configuration Update Failed', error?.message || 'Request failed')
    }
}

export let elrsState = new ElrsState()
