import {html} from 'lit'
import {customElement, property} from 'lit/decorators.js'
import {GyroElement} from './gyro-element.js'

const LINKSTATE = ['Connected', 'Tentative', 'Awaiting ModelID', 'Disconnected']
const AIRRATE = {
    0: '25Hz', 1: '50Hz', 2: '100Hz', 3: '100Hz Full', 4: '150Hz', 5: '200Hz',
    6: '200Hz Full', 7: '250Hz', 8: '333Hz Full', 9: '500Hz', 10: 'D50Hz',
    11: 'K1000Hz Full', 20: '25Hz', 21: '50Hz', 22: '100Hz', 23: '100Hz Full',
    24: '150Hz', 25: '200Hz', 26: '200Hz Full', 27: '250Hz', 28: '333Hz Full',
    29: '500Hz', 30: 'D250Hz', 31: 'D500Hz', 32: 'F500Hz', 33: 'F1000Hz',
    34: 'DK250Hz', 35: 'DK500Hz', 36: 'K1000Hz', 100: 'X100Hz Full', 101: 'X150Hz',
}
const GYRO_STATUS = ['Off', 'Not detected', 'Need RX orientation', 'Need stick calibration', 'OK']
const STATUS_BITS = {LEVEL_CAL: 1 << 0, ORIENTATION: 1 << 1}

/**
 * Displays receiver gyro, IMU, link, and calibration status.
 *
 * Calibration buttons emit `gyro-calibration-open` with the requested wizard;
 * the top-level panel owns wizard exclusivity and receiver state refreshes.
 */
@customElement('gyro-status-card')
export class GyroStatusCard extends GyroElement {
    @property({attribute: false}) accessor config = {}
    @property({attribute: false}) accessor runtime = null
    @property({attribute: false}) accessor calibrationBusy = false

    _buttonClass(bit) {
        return ((Number(this.runtime?.status_bits) || 0) & bit) !== 0
            ? 'mui-btn mui-btn--small'
            : 'mui-btn mui-btn--small mui-btn--danger'
    }

    _openCalibration(wizard) {
        this.emit('gyro-calibration-open', {wizard})
    }

    render() {
        const runtime = this.runtime ?? {}
        return html`
            <div class="mui-panel gyro-card gyro-status-card">
                <div class="mui--text-title">Status</div>
                <table class="mui-table gyro-status-table">
                    <thead><tr><th>IMU</th><th>Gyro State</th><th>Link</th></tr></thead>
                    <tbody><tr>
                        <td><div class="gyro-status-detail"><div class="gyro-kv-value">${this.config.imu}</div><div class="gyro-kv-hint">re=${runtime['read-errors']} · ie=${runtime['int-errors']}</div></div></td>
                        <td><div class="gyro-status-detail"><div class="gyro-kv-value">${GYRO_STATUS[runtime.status] ?? ''}</div><div class="gyro-kv-hint">${runtime.next_action || runtime.error}</div></div></td>
                        <td><div class="gyro-status-detail"><div class="gyro-kv-value">${LINKSTATE[runtime.link] ?? ''}</div><div class="gyro-kv-hint">${AIRRATE[runtime.rate] ?? ''}</div></div></td>
                    </tr></tbody>
                </table>
                <div class="gyro-action-row">
                    <button class=${this._buttonClass(STATUS_BITS.ORIENTATION)} ?disabled=${this.calibrationBusy} @click=${() => this._openCalibration('orientation')}>Orientation Wizard</button>
                    <button class=${this._buttonClass(STATUS_BITS.LEVEL_CAL)} ?disabled=${this.calibrationBusy} @click=${() => this._openCalibration('level')}>Level Cal</button>
                </div>
            </div>
        `
    }
}
