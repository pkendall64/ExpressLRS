import {html, LitElement} from "lit"
import {customElement, state} from "lit/decorators.js"
import {elrsState, saveOptionsAndConfig} from "../utils/state.js"
import {SERIAL_OPTIONS1, SERIAL_OPTIONS2} from "../utils/globals.js"
import {PWM_MODE_SERIAL_RX, PWM_MODE_SERIAL_TX, PWM_MODE_SERIAL2RX, PWM_MODE_SERIAL2TX} from "./connections-panel.js"


@customElement('serial-panel')
class SerialPanel extends LitElement {

    PROTOCOL_AIRPORT = SERIAL_OPTIONS1.length - 1

    @state() accessor serial1Protocol
    @state() accessor serial2Protocol
    @state() accessor baudRate
    @state() accessor sbusFailsafe
    @state() accessor isAirport
    @state() accessor djiArmed

    createRenderRoot() {
        this.isAirport = elrsState.options['is-airport']
        this.serial1Protocol = this.isAirport ? this.PROTOCOL_AIRPORT : elrsState.config['serial-protocol']
        this.serial2Protocol = elrsState.config['serial1-protocol']
        this.baudRate = elrsState.options['rcvr-uart-baud']
        this.sbusFailsafe = elrsState.config['sbus-failsafe']
        this.djiArmed = elrsState.options['dji-permanently-armed']
        this._saveSerial = this._saveSerial.bind(this)
        this._setSbusFailsafe = this._setSbusFailsafe.bind(this)
        return this
    }

    render() {
        return html`
            <div class="mui-panel mui--text-title">Serial/UART Options</div>
            ${this._hasSerial1() || this._hasSerial2() ? html`
            <div class="mui-panel">
                <p>Set the protocol(s) used to communicate with the flight controller or other external devices.</p>
                <form>
                    ${this._hasSerial1() ? html`
                    <div class="mui-select">
                        <select name='serial-protocol' @change=${this._updateSerial1}>
                            ${this._renderProtocolOptions(SERIAL_OPTIONS1, this.serial1Protocol, this._serialProtocolMask())}
                        </select>
                        <label>Serial 1 Protocol</label>
                    </div>
                    ` : ''}
                    ${this._hasSerial2() ? html`
                    <div class="mui-select">
                        <select name='serial1-protocol' @change=${this._updateSerial2}>
                            ${this._renderProtocolOptions(SERIAL_OPTIONS2, this.serial2Protocol, this._serial1ProtocolMask())}
                        </select>
                        <label>Serial 2 Protocol</label>
                    </div>
                    ` : ''}
                    ${this._displayBaudRate() ? html`
                    <div class="mui-textfield">
                        <input size='7' type='number'
                               @input=${(e) => this.baudRate = parseInt(e.target.value)}
                               .value="${this.baudRate}" />
                        <label>CRSF/Airport baud</label>
                    </div>
                    ` : ''}
                    ${this._sbusSelected() ? html`
                    <div id="sbus-config">
                        <div class="mui--text-title">SBUS Failsafe</div>
                        Set the failsafe behaviour when using the SBUS protocol:<br/>
                        <ul>
                            <li>"No Pulses" stops sending SBUS data when a connection to the transmitter is lost
                            </li>
                            <li>"Last Position" continues to send the last received channel data along with the
                                FAILSAFE
                                bit set
                            </li>
                        </ul>
                        <br/>
                        <div class="mui-select">
                            <select name='serial-failsafe'
                                    @change=${this._setSbusFailsafe}>
                                ${_renderOptions(['No Pulses', 'Last Position'], this.sbusFailsafe)}
                            </select>
                            <label>SBUS Failsafe</label>
                        </div>
                    </div>
                    ` : ''}
                    ${this._displayPortSelected() ? html`
                    <div class="mui-checkbox">
                        <input id="dji" type='checkbox'
                               ?checked="${this.djiArmed}"
                               @change="${(e) => {this.djiArmed = e.target.checked}}"/>
                        <label for="dji">Permanently arm DJI air units</label>
                    </div>
                    ` : ''}
                    <button class="mui-btn mui-btn--small mui-btn--primary"
                            ?disabled="${!this.checkChanged()}"
                            @click="${this._saveSerial}"
                    >Save</button>
                </form>
            </div>
            `: html`
            <div class="mui-panel info-bg">
                ${this._canConfigureSerialOnPwm() || elrsState.settings.has_serial_pins ? 
                    html`This is a PWM receiver and none of the pins have been configured as serial IO pins.<br>
                    To enable serial IO, go to the <a href="#connections">connections</a> menu and configure one or more pins as Serial RX, Serial TX, Serial2 RX, or Serial2 TX.
                    ` : html`This receiver does not have any serial-capable PWM pins available.`
                }
            </div>
            `}
        `
    }

    _canConfigureSerialOnPwm() {
        if (!elrsState.config['pwm']) return false
        return elrsState.config.pwm.some((pwm) => (pwm.features & (3 | 96)) !== 0)
    }

    _pwmSerialDirections(rxMode, txMode) {
        let directions = elrsState.settings.has_serial_pins ? 3 : 0
        for (const pwm of elrsState.config.pwm || []) {
            const mode = (pwm.config >> 16) & 31
            if (mode === rxMode) directions |= 1
            if (mode === txMode) directions |= 2
        }
        return directions
    }

    _serialProtocolMask() {
        const mask = elrsState.settings['serial-protocol-mask']
        if (mask !== undefined) return Number(mask)

        const directions = this._pwmSerialDirections(PWM_MODE_SERIAL_RX, PWM_MODE_SERIAL_TX)
        let protocols = 0
        if ((directions & 3) === 3) protocols |= (1 << 0) | (1 << 1) | (1 << 7) | (1 << 10)
        if (directions & 2) protocols |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 8)
        if (directions & 1) protocols |= 1 << 9
        return protocols
    }

    _serial1ProtocolMask() {
        const mask = elrsState.settings['serial1-protocol-mask']
        if (mask !== undefined) return Number(mask)

        const directions = this._pwmSerialDirections(PWM_MODE_SERIAL2RX, PWM_MODE_SERIAL2TX)
        let protocols = 1 // Off
        if ((directions & 3) === 3) protocols |= (1 << 1) | (1 << 2)
        if (directions & 2) protocols |= (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10)
        if (directions & 1) protocols |= 1 << 11
        return protocols
    }

    _renderProtocolOptions(options, selected, available) {
        return options.map((label, index) =>
            available & (1 << index)
                ? html`<option .value="${index.toString()}" ?selected="${index === selected}">${label}</option>`
                : '')
    }

    _hasSerial1() {
        return this._serialProtocolMask() !== 0
    }

    _hasSerial2() {
        return this._serial1ProtocolMask() > 1
    }

    _updateSerial1(e) {
        this.serial1Protocol = Number(e.target.value)
        this.isAirport = this.serial1Protocol === this.PROTOCOL_AIRPORT
        if (this.serial1Protocol === 0 || this.serial1Protocol === 1) {
            this.baudRate = 420000
            this.requestUpdate()
        }
    }

    _updateSerial2(e) {
        this.serial2Protocol = Number(e.target.value)
    }

    _setSbusFailsafe(e) {
        this.sbusFailsafe = Number(e.target.value)
    }

    _displayBaudRate() {
        return (this._hasSerial1() && (this.isAirport || this.serial1Protocol === 0 || this.serial1Protocol === 1)) ||
            (this._hasSerial2() && (this.serial2Protocol === 1 || this.serial2Protocol === 2))
    }

    _sbusSelected() {
        return (this._hasSerial1() && (this.serial1Protocol === 2 || this.serial1Protocol === 3)) ||
            (this._hasSerial2() && (this.serial2Protocol === 3 || this.serial2Protocol === 4))
    }

    _displayPortSelected() {
        return (this._hasSerial1() && this.serial1Protocol === 8) ||
            (this._hasSerial2() && this.serial2Protocol === 9)
    }

    _configChanged() {
        return (!this.isAirport && this.serial1Protocol !== elrsState.config['serial-protocol']) ||
            this.serial2Protocol !== elrsState.config['serial1-protocol'] ||
            this.sbusFailsafe !== elrsState.config['sbus-failsafe']
    }
    _optionsChanged() {
        return this.isAirport !== elrsState.options['is-airport'] ||
            this.baudRate !== elrsState.options['rcvr-uart-baud'] ||
            this.djiArmed !== elrsState.options['dji-permanently-armed']
    }

    checkChanged() {
        return this._configChanged() || this._optionsChanged()
    }

    _saveSerial(e) {
        e.preventDefault()
        saveOptionsAndConfig({
                options: {
                    'is-airport': this.isAirport,
                    'rcvr-uart-baud': this.baudRate,
                    'dji-permanently-armed': this.djiArmed,
                },
                config: {
                    'serial-protocol': this.isAirport ? 0 : this.serial1Protocol,
                    'serial1-protocol': this.serial2Protocol,
                    'sbus-failsafe': this.sbusFailsafe
                }
            },
            () => {this.requestUpdate()}
        )
    }
}
