import {html} from 'lit'
import {customElement, property, state} from 'lit/decorators.js'
import {GyroElement} from './gyro-element.js'
import {showAlert} from '../../utils/feedback.js'

const GYRO_MODE_LABELS = new Map([
    [0, 'Off'], [1, 'Rate'], [2, 'Envelope'], [3, 'Auto-Level'], [4, 'Launch'], [5, 'Hover'],
])
const STICK_PRIORITY_OPTIONS = [
    {id: 0, label: '100%'},
    {id: 1, label: '75%'},
    {id: 2, label: '50%'},
    {id: 3, label: '25%'},
]
const GAIN_FACTOR_OPTIONS = [
    {id: 0, label: '0.5×'},
    {id: 1, label: '1×'},
    {id: 2, label: '1.5×'},
    {id: 3, label: '2×'},
]
const GYRO_MODE_FIELD_LIMITS = {
    gainRoll: {min: 0, max: 250},
    gainPitch: {min: 0, max: 250},
    gainYaw: {min: 0, max: 250},
    rollLimit: {min: 30, max: 90},
    pitchLimit: {min: 10, max: 50},
    trimRoll: {min: -30, max: 30},
    trimPitch: {min: -30, max: 30},
}

function gyroModeFieldIsValid(mode, key) {
    const limits = GYRO_MODE_FIELD_LIMITS[key]
    return Number.isFinite(mode[key]) && mode[key] >= limits.min && mode[key] <= limits.max
}

function gyroModeIsValid(mode) {
    const fields = ['gainRoll', 'gainPitch', 'gainYaw']
    if (gyroModeShowsAngleLimits(mode.modeId)) fields.push('rollLimit', 'pitchLimit')
    if (gyroModeShowsTrims(mode.modeId)) fields.push('trimRoll', 'trimPitch')
    return fields.every((key) => gyroModeFieldIsValid(mode, key))
        && GAIN_FACTOR_OPTIONS.some((option) => option.id === mode.gainFactor)
        && (!gyroModeShowsStickPriority(mode.modeId)
            || STICK_PRIORITY_OPTIONS.some((option) => option.id === mode.stickPriority))
}

function getGyroModeLabel(modeId) {
    return GYRO_MODE_LABELS.get(Number(modeId)) ?? 'Off'
}

function gyroModeShowsUseRate(modeId) {
    return modeId !== 1
}

function gyroModeShowsStickPriority(modeId) {
    return modeId === 1
}

function gyroModeShowsAngleLimits(modeId) {
    return modeId === 2 || modeId === 3
}

function gyroModeShowsTrims(modeId) {
    return modeId === 3 || modeId === 4
}

/**
 * Edits all active gyro-mode tuning profiles as one transaction.
 *
 * Input changes remain local until Save posts the complete `gyro_modes` array.
 * Emits `gyro-config-updated` after save and `gyro-config-reset` when Reset asks
 * the parent to discard local edits and reload receiver state.
 */
@customElement('gyro-modes-card')
export class GyroModesCard extends GyroElement {
    @property({attribute: false}) accessor gyroModes = []
    @property({attribute: false}) accessor activeModeId = 0
    @state() accessor saving = false

    _updateGyroMode(index, key, value) {
        const next = [...this.gyroModes]
        next[index] = {...next[index], [key]: value}
        this.gyroModes = next
        this.emit('gyro-modes-draft', next)
    }

    async _saveGyroModes() {
        if (!this.gyroModes.every(gyroModeIsValid)) {
            await showAlert('error', 'Invalid Gyro Mode', 'Expected gain multipliers 0.5–2×, gains 0–250%, pitch limits 10–50°, roll limits 30–90°, and trims -30–30°.')
            return
        }

        this.saving = true
        try {
            await this.saveConfig({gyro_modes: this.gyroModes})
        } catch (error) {
            await showAlert('error', 'Gyro Mode Save Failed', error?.message || 'Request failed')
        } finally {
            this.saving = false
        }
    }

    _renderField(index, mode, key, label, unit) {
        const limits = GYRO_MODE_FIELD_LIMITS[key]
        return html`
            <div class="mui-textfield gyro-unit-field">
                <input id=${`gyro-${key}-${index}`} type="number" min=${String(limits.min)}
                       max=${String(limits.max)} step="1" required .value=${String(mode[key])}
                       @input=${(event) => this._updateGyroMode(index, key, event.target.value === '' ? Number.NaN : Number(event.target.value))}>
                <span class="gyro-unit-suffix">${unit}</span>
                <label for=${`gyro-${key}-${index}`}>${label}</label>
            </div>
        `
    }

    _renderGyroModeCard(mode, index) {
        return html`
            <div class="mui-panel gyro-mode-card ${mode.modeId === this.activeModeId ? 'active' : ''}">
                <div class="mui--text-title">${getGyroModeLabel(mode.modeId)}</div>
                <form class="mui-form gyro-form"><div class="gyro-stack">
                    ${gyroModeShowsStickPriority(mode.modeId) ? html`
                        <div class="mui-select">
                            <select id=${`gyro-gain-factor-${index}`}
                                    @change=${(event) => this._updateGyroMode(index, 'gainFactor', Number(event.target.value))}>
                                ${GAIN_FACTOR_OPTIONS.map((option) => html`
                                    <option value=${String(option.id)} ?selected=${option.id === mode.gainFactor}>${option.label}</option>`)}
                            </select>
                            <label for=${`gyro-gain-factor-${index}`}>Gain Multiplier</label>
                        </div>` : ''}
                    ${gyroModeShowsUseRate(mode.modeId) ? html`<div class="mui-checkbox"><input id=${`gyro-use-rate-${index}`} type="checkbox" ?checked=${mode.useRate} @change=${(event) => this._updateGyroMode(index, 'useRate', event.target.checked)}><label for=${`gyro-use-rate-${index}`}>Use Rate</label></div>` : ''}
                    ${gyroModeShowsStickPriority(mode.modeId) ? html`<div class="mui-select"><select id=${`gyro-priority-${index}`} .value=${String(mode.stickPriority)} @change=${(event) => this._updateGyroMode(index, 'stickPriority', Number(event.target.value))}>${STICK_PRIORITY_OPTIONS.map((option) => html`<option value=${String(option.id)}>${option.label}</option>`)}</select><label for=${`gyro-priority-${index}`}>Stick Priority</label></div>` : ''}
                    <div class="gyro-group-block"><div class="gyro-group-title">Gains</div><div class="gyro-grid gyro-inline-row gyro-inline-row--triple">
                        ${this._renderField(index, mode, 'gainRoll', 'Roll', '%')}
                        ${this._renderField(index, mode, 'gainPitch', 'Pitch', '%')}
                        ${this._renderField(index, mode, 'gainYaw', 'Yaw', '%')}
                    </div></div>
                    ${gyroModeShowsAngleLimits(mode.modeId) ? html`<div class="gyro-group-block"><div class="gyro-group-title">Limits</div><div class="gyro-grid gyro-inline-row gyro-inline-row--double">
                        ${this._renderField(index, mode, 'rollLimit', 'Roll', '°')}
                        ${this._renderField(index, mode, 'pitchLimit', 'Pitch', '°')}
                    </div></div>` : ''}
                    ${gyroModeShowsTrims(mode.modeId) ? html`<div class="gyro-group-block"><div class="gyro-group-title">Trims</div><div class="gyro-grid gyro-inline-row gyro-inline-row--double">
                        ${this._renderField(index, mode, 'trimRoll', 'Roll', '°')}
                        ${this._renderField(index, mode, 'trimPitch', 'Pitch', '°')}
                    </div></div>` : ''}
                </div></form>
            </div>
        `
    }

    render() {
        return html`
            <div class="gyro-tuning-card">
                <div class="gyro-grid gyro-grid--modes">${this.gyroModes.map((mode, index) => this._renderGyroModeCard(mode, index))}</div>
                <div class="gyro-save-row">
                    <button class="mui-btn mui-btn--primary mui-btn--small" ?disabled=${this.saving} @click=${() => this._saveGyroModes()}>${this.saving ? 'Saving…' : 'Save Gyro Modes'}</button>
                    <button class="mui-btn mui-btn--small" ?disabled=${this.saving} @click=${() => this.emit('gyro-config-reset')}>Reset</button>
                </div>
            </div>
        `
    }
}
