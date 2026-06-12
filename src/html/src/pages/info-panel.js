import {html, LitElement} from "lit"
import {customElement} from "lit/decorators.js"
import {elrsState, formatBand, formatWifiRssi} from "../utils/state.js"
import {SERIAL_OPTIONS1} from '../utils/globals.js'

@customElement('info-panel')
class InfoPanel extends LitElement {
    createRenderRoot() {
        return this
    }

    INFO_PROPS = {
        'Product': { v: elrsState.settings.product_name },
        'Lua Name': { v: elrsState.settings.lua_name },
        'Version':  { v: elrsState.settings.version },
        'Git Hash': { v: elrsState.settings['git-commit'] },
        'Device Type': { v: elrsState.settings['module-type'] },
        'Firmware': { v: elrsState.settings.target },
        'Radio': { v: elrsState.settings['radio-type'] },
        'Domain':  { v: formatBand() },
        'Binding UID': { v: elrsState.config.uid.toString() },
        'WiFi State': { v: formatWifiRssi() },
    }

    CUSTOM_PROPS = {
        'Airport Mode': {
            v: 'Enabled',
            w: elrsState.options['is-airport']
        },
        'Wifi Auto-on Interval': {
            v: elrsState.options['wifi-on-interval'],
            w: elrsState.options['wifi-on-interval'] !== 60
        },
        // FEATURE: NOT IS_TX
        'Lock on First Connection': {
            v: 'False',
            w: elrsState.options['lock-on-first-connection'] !== true
        },
        'Model Match': {
            v: `Enabled (ID: ${elrsState.config.modelid})`,
            w: elrsState.config.modelid !== 255
        },
        'Binding Storage': {
            v: elrsState.config.vbind === 1 ? 'Volatile' : elrsState.config.vbind === 2 ? 'Returnable' : 'Administered',
            w: elrsState.config.vbind !== 0
        },
        'Force Telemetry Off': {
            v: 'Enabled',
            w: elrsState.config['force-tlm'] !== false
        },
        'Serial Protocol': {
            v: SERIAL_OPTIONS1[elrsState.config['serial-protocol']],
            w: elrsState.config['pwm'] === undefined && elrsState.config['serial-protocol'] !== 0
        },
        'Baud Rate': {
            v: elrsState.options['rcvr-uart-baud'],
            w: elrsState.config['pwm'] === undefined && elrsState.options['rcvr-uart-baud'] !== 420000
        },
        // /FEATURE: NOT IS_TX
        // FEATURE: IS_TX
        'Telemetry Report Interval (ms)': {
            v: elrsState.options['tlm-interval'],
            w: elrsState.options['tlm-interval'] !== 240
        },
        // /FEATURE: IS_TX
        'Customised Hardware Settings': {
            v: 'True',
            w: elrsState.settings?.custom_hardware
       }
    }

    formatCustomProps(props) {
        return Object.entries(props).map(([key, value]) =>
                value?.w === undefined || value.w ?
                    html`<tr><td><b>${key}</b></td><td>${value.v}</td></tr>`
                    : ''
        )
    }

    hasCustomisations() {
        return Object.entries(this.CUSTOM_PROPS).find(([key, value]) => value.w)
    }

    render() {
        return html`
            <div class="mui-panel mui--text-title">Information</div>
            <div class="mui-panel">
                <table class="mui-table mui-table--bordered">
                    <tbody>
                    ${this.formatCustomProps(this.INFO_PROPS)}
                    </tbody>
                </table>
            </div>
            ${this.hasCustomisations() ? html`
                <div class="mui-panel warning-bg">
                    <div class="mui--text-title">Custom Settings Detected</div>
                    <br>
                    <table class="mui-table mui-table--bordered">
                        <tbody>
                        ${this.formatCustomProps(this.CUSTOM_PROPS)}
                        </tbody>
                    </table>
                </div>
                `:
                ''
            }
        `
    }
}
