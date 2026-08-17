import {html} from 'lit'
import {customElement, property, state} from 'lit/decorators.js'
import {GyroElement} from './gyro-element.js'
import {showAlert} from '../../utils/feedback.js'

const GYRO_MODE_OPTIONS = [{id: 0, label: 'Off'}, {id: 1, label: 'Rate'}, {id: 2, label: 'Envelope'}, {
    id: 3,
    label: 'Auto-Level'
}, {id: 4, label: 'Launch'}, {id: 5, label: 'Hover'},]
const MODE_SWITCH_POSITION_OPTIONS = [2, 3, 4, 5, 6]

// Mirrors fmap() and CRSF_to_N() in crsf_protocol.h.
function usToCrsf(us) {
    return Math.round((us - 988) * (1811 - 172) / (2012 - 988) + 172)
}

function crsfToN(value, count) {
    if (value <= 191) return 0
    if (value >= 1792) return count - 1
    return Math.floor((value - 191) * count / (1792 - 191 + 1))
}

function formatModeSwitchRange(positions, index) {
    if (positions < 2 || index < 0 || index >= positions) return ''

    let min = 1000
    while (crsfToN(usToCrsf(min), positions) < index) min++
    let max = min
    while (max < 2000 && crsfToN(usToCrsf(max + 1), positions) === index) max++
    return `${min}–${max} µs`
}

/**
 * Edits gyro enablement and the configured mode switch map.
 *
 * Each control is persisted immediately. Successful writes include the changed
 * fields so edits in other cards remain untouched.
 */
@customElement('gyro-main-setup-card')
export class GyroMainSetupCard extends GyroElement {
    @property({attribute: false}) accessor config = {}
    @property({attribute: false}) accessor modeMap = []
    @property({attribute: false}) accessor activeModePosition = -1
    @property({attribute: false}) accessor modeSwitchPositions = 3
    @property({attribute: false}) accessor hasModeChannel = false
    @state() accessor modeMapSaving = false
    @state() accessor enabledSaving = false

    async _saveModeMap(next) {
        this.modeMap = next
        this.modeMapSaving = true
        try {
            await this.saveConfig({mode_map: next})
        } catch (error) {
            await showAlert('error', 'Gyro Mode Map Save Failed', error?.message || 'Request failed')
        } finally {
            this.modeMapSaving = false
        }
    }

    async _saveModeSwitchPositions(positions) {
        this.modeSwitchPositions = positions
        const modeMap = this.modeMap.slice(0, positions)
        while (modeMap.length < positions) modeMap.push(0)
        this.modeMap = modeMap
        this.modeMapSaving = true
        try {
            await this.saveConfig({mode_switch_positions: positions, mode_map: modeMap})
        } catch (error) {
            await showAlert('error', 'Gyro Mode Switch Save Failed', error?.message || 'Request failed')
        } finally {
            this.modeMapSaving = false
        }
    }

    async _saveEnabled(enabled) {
        this.config = {...this.config, enabled}
        this.enabledSaving = true
        try {
            await this.saveConfig({enabled})
        } catch (error) {
            await showAlert('error', 'Gyro Enable Save Failed', error?.message || 'Request failed')
        } finally {
            this.enabledSaving = false
        }
    }


    render() {
        const activeModeSlot = this.hasModeChannel ? this.activeModePosition : -1
        return html`
            <div class="mui-panel gyro-card gyro-main-card">
                <div class="mui--text-title">Main Setup</div>
                <form class="mui-form gyro-form">
                    <div class="gyro-grid gyro-grid--setup">
                        <div class="mui-checkbox">
                            <input id="gyro-enabled" type="checkbox" ?checked=${this.config.enabled}
                                   ?disabled=${this.enabledSaving}
                                   @change=${(event) => this._saveEnabled(event.target.checked)}>
                            <label for="gyro-enabled">Enable Gyro</label>
                        </div>
                    </div>
                </form>
                <div class="gyro-section-label">Gyro Mode Switch Map</div>
                ${this.hasModeChannel ? '' : html`
                    <div class="mui-panel warning-bg" role="alert">
                        No gyro mode switch channel is configured. Add a channel function mapping with the Gyro Mode function.
                    </div>
                `}
                <div class="mui-select">
                    <select id="gyro-mode-switch-positions" ?disabled=${this.modeMapSaving}
                            @change=${(event) => this._saveModeSwitchPositions(Number(event.target.value))}>
                        ${MODE_SWITCH_POSITION_OPTIONS.map((positions) => html`
                            <option value=${String(positions)} ?selected=${positions === this.modeSwitchPositions}>
                                ${positions} positions
                            </option>`)}
                    </select>
                    <label for="gyro-mode-switch-positions">Switch type</label>
                </div>
                <div class="gyro-grid gyro-grid--mode-map">
                    ${this.modeMap.slice(0, this.modeSwitchPositions).map((modeId, index) => html`
                        <div class="mui-panel gyro-mode-slot ${index === activeModeSlot ? 'active' : ''}">
                            <div class="pos">Pos ${index + 1}</div>
                            <div class="gyro-kv-hint">${formatModeSwitchRange(this.modeSwitchPositions, index)}</div>
                            <div class="mui-select">
                                <select id=${`gyro-pos-${index}`} ?disabled=${this.modeMapSaving} @change=${(event) => {
                                    const next = [...this.modeMap]
                                    next[index] = Number(event.target.value)
                                    this._saveModeMap(next)
                                }}>
                                    ${GYRO_MODE_OPTIONS.map((option) => html`
                                        <option value=${String(option.id)} ?selected=${option.id === modeId}>
                                            ${option.label}
                                        </option>`)}
                                </select>
                                <label for=${`gyro-pos-${index}`}>Mode</label>
                            </div>
                        </div>
                    `)}
                </div>
            </div>
        `
    }
}
