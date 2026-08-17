import {html} from "lit"
import {customElement, query, state} from "lit/decorators.js"
import {loadJSON, showAlert, showConfirm} from "../utils/feedback.js"
import {GyroElement} from "../components/gyro/gyro-element.js"
import "../components/gyro/gyro-attitude-card.js"
import "../components/gyro/gyro-tuning-tabs.js"
import "../components/gyro/gyro-level-calibration-wizard.js"
import "../components/gyro/gyro-main-setup-card.js"
import "../components/gyro/gyro-orientation-wizard.js"
import "../components/gyro/gyro-channel-functions-card.js"
import "../components/gyro/gyro-status-card.js"
import "../components/gyro/gyro-stick-calibration-wizard.js"

const STICK_CAL_BIT = 1 << 2

/**
 * Coordinates the gyro feature's durable configuration and live runtime data.
 *
 * Cards receive snapshots through properties and report successful writes with
 * bubbling events. This panel merges those writes without discarding other drafts.
 * Runtime attitude and status data use a reconnecting WebSocket, while only one
 * calibration wizard may own the receiver calibration state at a time.
 *
 * The panel intentionally uses light DOM so global MUI and gyro.css rules apply.
 */
@customElement('gyro-panel')
class GyroPanel extends GyroElement {
    @state() accessor gyroConfig = {}
    @state() accessor gyroRuntime = null
    @state() accessor calibrationWizard = null
    @query('#gyro-config-import') accessor configImport
    @state() accessor importingConfig = false
    runtimeSocket = null
    runtimeReconnectTimer = null
    runtimeStopped = false

    async loadConfig() {
        this.gyroConfig = await loadJSON('/gyro-config.json')
    }

    _applyConfigUpdate(event) {
        const changes = event.detail
        if (changes.quick_setup) {
            this.loadConfig()
            return
        }
        this.gyroConfig = {...this.gyroConfig, ...changes}
    }

    _applyGyroModesDraft(event) {
        this.gyroConfig = {...this.gyroConfig, gyro_modes: event.detail}
    }

    _applyGyroPidsDraft(event) {
        this.gyroConfig = {...this.gyroConfig, gyro_pids: event.detail}
    }

    _connectRuntime() {
        if (this.runtimeStopped || this.runtimeSocket?.readyState === WebSocket.CONNECTING || this.runtimeSocket?.readyState === WebSocket.OPEN) return

        const protocol = location.protocol === 'https:' ? 'wss' : 'ws'
        const socket = new WebSocket(`${protocol}://${location.host}/gyro-runtime`)
        this.runtimeSocket = socket
        socket.onmessage = (event) => {
            try {
                this.gyroRuntime = JSON.parse(event.data)
            } catch (_error) {
            }
        }
        socket.onclose = () => {
            if (this.runtimeSocket === socket) this.runtimeSocket = null
            if (!this.runtimeStopped) this.runtimeReconnectTimer = setTimeout(() => this._connectRuntime(), 1000)
        }
    }

    async connectedCallback() {
        super.connectedCallback()
        this.runtimeStopped = false
        await this.loadConfig()
        try {
            this.gyroRuntime = await loadJSON('/gyro.json')
        } catch (_error) {
        }
        this._connectRuntime()
    }

    disconnectedCallback() {
        this.runtimeStopped = true
        clearTimeout(this.runtimeReconnectTimer)
        this.runtimeReconnectTimer = null
        this.runtimeSocket?.close()
        this.runtimeSocket = null
        super.disconnectedCallback()
    }

    _hasStatusBit(bit) {
        return ((Number(this.gyroRuntime?.status_bits) || 0) & bit) !== 0
    }

    _openCalibrationWizard(wizard) {
        if (!this.calibrationWizard) this.calibrationWizard = wizard
    }

    _closeCalibrationWizard() {
        this.calibrationWizard = null
    }

    async _finishCalibrationWizard() {
        this.calibrationWizard = null
        await this.loadConfig()
        this.gyroRuntime = await loadJSON('/gyro.json')
    }

    async _importConfig(event) {
        const file = event.currentTarget.files[0]
        event.currentTarget.value = ''
        if (!file) return

        const confirmed = await showConfirm(
            'Import Gyro Config',
            'Replace the current main setup, channel-function mappings, mode tuning, and advanced PIDs?',
            'Import',
            'Cancel',
        )
        if (confirmed !== 'confirm') return

        this.importingConfig = true
        try {
            const response = await fetch('/gyro-config.json?import', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: await file.text(),
            })
            if (!response.ok) throw new Error(await response.text() || 'Import failed')
            await this.loadConfig()
            await showAlert('success', 'Gyro Config Imported', 'The gyro configuration has been updated.')
        } catch (error) {
            await showAlert('error', 'Gyro Config Import Failed', error?.message || 'Import failed')
        } finally {
            this.importingConfig = false
        }
    }

    render() {
        const gyroConfig = this.gyroConfig
        const gyro = this.gyroRuntime ?? {}
        const activeGyroModeId = Number(gyro.mode)
        const activeModePosition = Number(gyro.mode_position)
        const hasModeChannel = (gyroConfig.channel_functions ?? []).some((channelFunction) => Number(channelFunction.functionId) === 8)

        return html`
            <div class="gyro-layout">
                <div class="mui-panel gyro-page-title gyro-page-header">
                    <div class="mui--text-title">Gyro</div>
                    <div class="gyro-page-actions">
                        <a class="mui-btn mui-btn--small" href="/gyro-config.json?export"
                           download="gyro-config.json">Export Config</a>
                        <button class="mui-btn mui-btn--small" type="button" ?disabled=${this.importingConfig}
                                @click=${() => this.configImport.click()}>
                            ${this.importingConfig ? 'Importing…' : 'Import Config'}
                        </button>
                        <input id="gyro-config-import" type="file" accept="application/json,.json" hidden
                               @change=${this._importConfig}>
                    </div>
                </div>
                <div class="gyro-top-grid">
                    <div class="gyro-stack gyro-left-stack">
                        <gyro-main-setup-card
                                class="gyro-main-card"
                                .config=${gyroConfig}
                                .modeMap=${gyroConfig.mode_map ?? []}
                                .activeModePosition=${activeModePosition}
                                .modeSwitchPositions=${gyroConfig.mode_switch_positions ?? 3}
                                .hasModeChannel=${hasModeChannel}
                                @gyro-config-updated=${this._applyConfigUpdate}>
                        </gyro-main-setup-card>
                        <gyro-channel-functions-card
                                class="gyro-channel-functions-card"
                                .channelFunctions=${gyroConfig.channel_functions ?? []}
                                .runtime=${this.gyroRuntime}
                                .calibrationNeeded=${!this._hasStatusBit(STICK_CAL_BIT)}
                                .calibrationBusy=${Boolean(this.calibrationWizard)}
                                @gyro-config-updated=${this._applyConfigUpdate}
                                @gyro-calibration-open=${(event) => this._openCalibrationWizard(event.detail.wizard)}>
                        </gyro-channel-functions-card>
                    </div>
                    <div class="gyro-stack gyro-right-stack">
                        <gyro-status-card
                                class="gyro-status-card"
                                .config=${gyroConfig}
                                .runtime=${this.gyroRuntime}
                                .calibrationBusy=${Boolean(this.calibrationWizard)}
                                @gyro-calibration-open=${(event) => this._openCalibrationWizard(event.detail.wizard)}>
                        </gyro-status-card>
                        <gyro-attitude-card class="gyro-attitude-card"
                                            .runtime=${this.gyroRuntime}></gyro-attitude-card>
                    </div>
                </div>
                <gyro-tuning-tabs
                        .gyroModes=${gyroConfig.gyro_modes ?? []}
                        .gyroPids=${gyroConfig.gyro_pids ?? []}
                        .activeModeId=${activeGyroModeId}
                        @gyro-config-updated=${this._applyConfigUpdate}
                        @gyro-modes-draft=${this._applyGyroModesDraft}
                        @gyro-pids-draft=${this._applyGyroPidsDraft}
                        @gyro-config-reset=${() => this.loadConfig()}>
                </gyro-tuning-tabs>
            </div>
            ${this.calibrationWizard === 'orientation' ? html`
                <gyro-orientation-wizard
                    @gyro-calibration-complete=${() => this._finishCalibrationWizard()}
                    @gyro-calibration-cancelled=${() => this._closeCalibrationWizard()}>
                </gyro-orientation-wizard>
            ` : this.calibrationWizard === 'level' ? html`
                <gyro-level-calibration-wizard
                    @gyro-calibration-complete=${() => this._finishCalibrationWizard()}
                    @gyro-calibration-cancelled=${() => this._closeCalibrationWizard()}>
                </gyro-level-calibration-wizard>
            ` : this.calibrationWizard === 'sticks' ? html`
                <gyro-stick-calibration-wizard
                    .channels=${gyro.channels ?? []}
                    .channelFunctions=${gyroConfig.channel_functions ?? []}
                    .limits=${gyro.stick_limits ?? []}
                    @gyro-calibration-complete=${() => this._finishCalibrationWizard()}
                    @gyro-calibration-cancelled=${() => this._closeCalibrationWizard()}>
                </gyro-stick-calibration-wizard>
            ` : ''}
        `
    }
}
