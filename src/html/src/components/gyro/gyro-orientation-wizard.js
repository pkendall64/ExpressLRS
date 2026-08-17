import {html} from 'lit'
import {customElement, state} from 'lit/decorators.js'
import {GyroElement} from './gyro-element.js'
import {postJSON, withRequestState} from '../../utils/feedback.js'
import '../wizard-dialog.js'

const ORIENTATION_BIT = 1 << 1

/**
 * Drives the receiver's two-step horizontal/nose-down orientation state machine.
 *
 * Once the horizontal sample succeeds, closing or disconnecting must send
 * `cancel` so firmware does not remain in an intermediate calibration state.
 */
@customElement('gyro-orientation-wizard')
export class GyroOrientationWizard extends GyroElement {
    @state() accessor step = 0
    @state() accessor busy = false
    @state() accessor error = ''

    // True only while firmware is waiting for the second orientation sample.
    needsCancel = false

    disconnectedCallback() {
        if (this.needsCancel) {
            postJSON('/gyro-calibration.json', {action: 'cancel'}, {keepalive: true, raw: true}).catch(() => {
            })
        }
        super.disconnectedCallback()
    }

    _action(action) {
        return withRequestState(this, () => postJSON('/gyro-calibration.json', {action}, {errorMessage: 'Calibration request failed'}), 'Calibration request failed')
    }

    async _captureLevel() {
        if (await this._action('orientation-horizontal')) {
            this.needsCancel = true
            this.step = 1
        }
    }

    async _captureNoseDown() {
        const state = await this._action('orientation-vertical')
        if (!state) return
        this.needsCancel = false
        if ((state.status_bits & ORIENTATION_BIT) === 0) {
            this.step = 0
            this.error = 'Orientation could not be detected. Keep the receiver still and start again.'
            return
        }
        this.emit('gyro-calibration-complete')
    }

    async _cancel() {
        if (this.needsCancel && !(await this._action('cancel'))) return
        this.needsCancel = false
        this.emit('gyro-calibration-cancelled')
    }

    render() {
        return html`
            <elrs-wizard-dialog
                title="Orientation Calibration"
                icon="icon--symbols icon--symbols--gyro"
                ?closeDisabled=${this.busy}
                @wizard-close=${() => this._cancel()}
                .notice=${this.error}
                noticeType="error"
                .body=${html`
                    ${this.step === 0 ? html`
                        <p>Place the aircraft and receiver level and keep them still.</p>
                        <div class="gyro-wizard-actions">
                            <button class="mui-btn mui-btn--primary mui-btn--small" ?disabled=${this.busy} @click=${() => this._captureLevel()}>Capture Level</button>
                        </div>
                    ` : html`
                        <p>Point the aircraft nose down and keep the receiver still.</p>
                        <div class="gyro-wizard-actions">
                            <button class="mui-btn mui-btn--primary mui-btn--small" ?disabled=${this.busy} @click=${() => this._captureNoseDown()}>Capture Nose Down</button>
                        </div>
                    `}
                `}>
            </elrs-wizard-dialog>
        `
    }
}
