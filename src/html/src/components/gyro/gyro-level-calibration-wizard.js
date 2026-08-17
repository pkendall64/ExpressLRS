import {html} from 'lit'
import {customElement, state} from 'lit/decorators.js'
import {GyroElement} from './gyro-element.js'
import {postJSON, withRequestState} from '../../utils/feedback.js'
import '../wizard-dialog.js'

const LEVEL_CAL_BIT = 1 << 0

/**
 * Runs the receiver's one-shot level calibration.
 *
 * Emits `gyro-calibration-complete` only after the receiver reports the level
 * status bit, or `gyro-calibration-cancelled` when the dialog closes.
 */
@customElement('gyro-level-calibration-wizard')
export class GyroLevelCalibrationWizard extends GyroElement {
    @state() accessor busy = false
    @state() accessor error = ''

    async _calibrate() {
        const state = await withRequestState(this, async () => {
            const state = await postJSON('/gyro-calibration.json', {action: 'level'}, {errorMessage: 'Calibration request failed'})
            if ((state.status_bits & LEVEL_CAL_BIT) === 0) {
                throw new Error('Level calibration did not complete. Keep the aircraft still and try again.')
            }
            return state
        }, 'Calibration request failed')
        if (state) this.emit('gyro-calibration-complete')
    }

    _cancel() {
        this.emit('gyro-calibration-cancelled')
    }

    render() {
        return html`
            <elrs-wizard-dialog
                title="Level Calibration"
                icon="icon--symbols icon--symbols--gyro"
                ?closeDisabled=${this.busy}
                @wizard-close=${() => this._cancel()}
                .body=${html`
                    <p>Place the aircraft level, keep it still, and ensure the propeller is safe before calibrating.</p>
                    ${this.error ? html`<p class="gyro-note">${this.error}</p>` : ''}
                    <div class="gyro-wizard-actions">
                        <button class="mui-btn mui-btn--primary mui-btn--small" ?disabled=${this.busy} @click=${() => this._calibrate()}>Calibrate Level</button>
                    </div>
                `}>
            </elrs-wizard-dialog>
        `
    }
}
