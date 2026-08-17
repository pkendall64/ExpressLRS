import {html} from 'lit'
import {customElement, property, state} from 'lit/decorators.js'
import {GyroElement} from './gyro-element.js'
import {postJSON, withRequestState} from '../../utils/feedback.js'
import '../wizard-dialog.js'
import './gyro-stick-range.js'

const STICK_CAL_BIT = 1 << 2
const MOVING_SURFACE_FUNCTIONS = new Set([1, 2, 3, 4, 5, 6, 7])

/**
 * Drives center capture followed by full-range capture for mapped surfaces.
 *
 * Center collection completes asynchronously in firmware. A 409 response while
 * starting range capture means "not ready yet" and is retried until completion
 * or cancellation. Any active sequence is cancelled when the component leaves.
 */
@customElement('gyro-stick-calibration-wizard')
export class GyroStickCalibrationWizard extends GyroElement {
    @property({attribute: false}) accessor channels = []
    @property({attribute: false}) accessor channelFunctions = []
    @property({attribute: false}) accessor limits = []
    @state() accessor step = 0
    @state() accessor busy = false
    @state() accessor error = ''
    @state() accessor waitingForCenters = false

    // Controls both the retry loop and whether disconnect must cancel firmware.
    needsCancel = false

    disconnectedCallback() {
        if (this.needsCancel) {
            this.needsCancel = false
            postJSON('/gyro-calibration.json', {action: 'cancel'}, {keepalive: true, raw: true}).catch(() => {
            })
        }
        super.disconnectedCallback()
    }

    _action(action) {
        return withRequestState(this, () => postJSON('/gyro-calibration.json', {action}, {errorMessage: 'Calibration request failed'}), 'Calibration request failed')
    }

    async _captureCenter() {
        if (await this._action('sticks-center')) {
            this.needsCancel = true
            this.waitingForCenters = true
            this._startRangeCapture()
        }
    }

    // Firmware uses 409 while it is still collecting centered-stick samples.
    async _startRangeCapture() {
        try {
            while (this.needsCancel) {
                const response = await postJSON('/gyro-calibration.json', {action: 'sticks-range-start'}, {raw: true})
                if (response.ok) {
                    this.waitingForCenters = false
                    this.step = 1
                    return
                }
                if (response.status !== 409) {
                    this.waitingForCenters = false
                    this.error = await response.text() || 'Calibration request failed'
                    return
                }
                await new Promise((resolve) => setTimeout(resolve, 50))
            }
        } catch (error) {
            this.waitingForCenters = false
            this.error = error?.message || 'Calibration request failed'
        }
    }

    async _finish() {
        const state = await this._action('sticks-range-finish')
        if (!state) return
        this.needsCancel = false
        if ((state.status_bits & STICK_CAL_BIT) === 0) {
            this.step = 0
            this.error = 'A mapped control did not move far enough. Start the calibration again.'
            return
        }
        this.emit('gyro-calibration-complete')
    }

    async _cancel() {
        if (this.needsCancel && !(await this._action('cancel'))) return
        this.needsCancel = false
        this.waitingForCenters = false
        this.emit('gyro-calibration-cancelled')
    }

    _renderRange(channelFunction, showEndpoints) {
        return html`<gyro-stick-range
            .channel=${channelFunction.channel}
            .functionName=${channelFunction.function}
            .limits=${this.limits[channelFunction.channel - 1] ?? {}}
            .input=${this.channels[channelFunction.channel - 1] || 1500}
            .showEndpoints=${showEndpoints}>
        </gyro-stick-range>`
    }

    render() {
        const calibratedChannelFunctions = this.channelFunctions.filter((channelFunction) => MOVING_SURFACE_FUNCTIONS.has(Number(channelFunction.functionId)))
        const showEndpoints = this.step === 1
        return html`
            <elrs-wizard-dialog
                title="Stick Calibration"
                icon="icon--symbols icon--symbols--gyro"
                ?closeDisabled=${this.busy}
                @wizard-close=${() => this._cancel()}
                .body=${html`
                    ${this.error ? html`<p class="gyro-note">${this.error}</p>` : ''}
                    ${this.step === 0 && !this.waitingForCenters ? html`
            <p>Center all sticks and trims, then capture their center positions.</p>
        ` : this.waitingForCenters ? html`
            <p>Capturing center positions. Keep all sticks and trims centered.</p>
        ` : html`
            <p>Move every mapped control through its full range and corners.</p>
        `}
                    <div class="gyro-stick-range-list">
                        ${calibratedChannelFunctions.length === 0 ? html`
            <div class="gyro-note">No moving-surface channel functions are mapped.</div>
        ` : calibratedChannelFunctions.map((channelFunction) => this._renderRange(channelFunction, showEndpoints))}
                    </div>
                    <div class="gyro-wizard-actions">
                        ${this.step === 0 && !this.waitingForCenters ? html`
            <button class="mui-btn mui-btn--primary mui-btn--small" ?disabled=${this.busy}
                    @click=${() => this._captureCenter()}>Capture Centers
            </button>
        ` : this.waitingForCenters ? '' : html`
            <button class="mui-btn mui-btn--primary mui-btn--small" ?disabled=${this.busy}
                    @click=${() => this._finish()}>Finish Calibration
            </button>
        `}
                    </div>
                `}>
            </elrs-wizard-dialog>
        `
    }
}
