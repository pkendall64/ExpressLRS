import {html} from 'lit'
import {customElement, property, state} from 'lit/decorators.js'
import {GyroElement} from './gyro-element.js'
import {showAlert} from '../../utils/feedback.js'

const PID_GROUPS = [
    {id: 0, label: 'Rate', scale: 'Values are divided by 100', axes: ['Roll', 'Pitch', 'Yaw']},
    {id: 1, label: 'Level', scale: 'Values are divided by 100', axes: ['Roll', 'Pitch', 'Yaw']},
    {id: 2, label: 'AHRS', scale: 'Values are divided by 10', axes: ['Filter']},
]

function pidLimits(groupId, key) {
    return key === 'p' && groupId === 2 ? {min: 10, max: 60} : {min: 0, max: 100}
}

function pidIsValid(pid) {
    return ['p', 'i', 'd'].every((key) => {
        const limits = pidLimits(pid.groupId, key)
        return Number.isInteger(pid[key]) && pid[key] >= limits.min && pid[key] <= limits.max
    })
}

/** Edits the rate, level, and AHRS PID groups as one transaction. */
@customElement('gyro-pids-card')
export class GyroPidsCard extends GyroElement {
    @property({attribute: false}) accessor gyroPids = []
    @state() accessor saving = false

    _updatePid(groupId, axisId, key, value) {
        this.gyroPids = this.gyroPids.map((pid) => pid.groupId === groupId && pid.axisId === axisId
            ? {...pid, [key]: value}
            : pid)
        this.emit('gyro-pids-draft', this.gyroPids)
    }

    async _savePids() {
        if (!this.gyroPids.every(pidIsValid)) {
            await showAlert('error', 'Invalid PID Settings', 'Expected PID values from 0–100; AHRS P must be 10–60.')
            return
        }

        this.saving = true
        try {
            await this.saveConfig({gyro_pids: this.gyroPids})
        } catch (error) {
            await showAlert('error', 'PID Save Failed', error?.message || 'Request failed')
        } finally {
            this.saving = false
        }
    }

    _renderInput(pid, key) {
        const limits = pidLimits(pid.groupId, key)
        return html`
            <div class="mui-textfield gyro-unit-field">
                <input id=${`gyro-pid-${pid.groupId}-${pid.axisId}-${key}`} type="number"
                       min=${String(limits.min)} max=${String(limits.max)} step="1" required
                       .value=${String(pid[key])} ?disabled=${this.saving}
                       @input=${(event) => this._updatePid(pid.groupId, pid.axisId, key,
                           event.target.value === '' ? Number.NaN : Number(event.target.value))}>
                <label for=${`gyro-pid-${pid.groupId}-${pid.axisId}-${key}`}>${key.toUpperCase()}</label>
            </div>
        `
    }

    render() {
        return html`
            <div class="gyro-tuning-card">
                <p class="gyro-note">Change these values only when tuning the gyro control loops.</p>
                <div class="gyro-grid gyro-grid--pids">
                    ${PID_GROUPS.map((group) => html`
                        <div class="mui-panel gyro-pid-card gyro-group-block">
                            <div>
                                <div class="gyro-group-title">${group.label}</div>
                                <div class="gyro-kv-hint">${group.scale}</div>
                            </div>
                            <div class="gyro-pid-grid">
                                ${this.gyroPids.filter((pid) => pid.groupId === group.id).map((pid) => html`
                                    <div class="gyro-pid-axis">${group.axes[pid.axisId]}</div>
                                    ${this._renderInput(pid, 'p')}
                                    ${this._renderInput(pid, 'i')}
                                    ${this._renderInput(pid, 'd')}
                                `)}
                            </div>
                        </div>
                    `)}
                </div>
                <div class="gyro-save-row">
                    <button class="mui-btn mui-btn--primary mui-btn--small" ?disabled=${this.saving}
                            @click=${() => this._savePids()}>${this.saving ? 'Saving…' : 'Save Advanced PIDs'}</button>
                    <button class="mui-btn mui-btn--small" ?disabled=${this.saving}
                            @click=${() => this.emit('gyro-config-reset')}>Reset</button>
                </div>
            </div>
        `
    }
}
