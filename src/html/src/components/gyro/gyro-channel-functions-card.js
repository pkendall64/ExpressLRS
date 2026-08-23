import {html} from 'lit'
import {customElement, property, state} from 'lit/decorators.js'
import {GyroElement} from './gyro-element.js'
import {postJSON, showAlert} from '../../utils/feedback.js'
import '../wizard-dialog.js'
import './gyro-stick-range.js'

const SURFACE_FUNCTION_OPTIONS = [
    {id: 1, label: 'Aileron'},
    {id: 2, label: 'Elevator'},
    {id: 3, label: 'Rudder'},
    {id: 4, label: 'Elevon'},
    {id: 5, label: 'Elevon R'},
    {id: 6, label: 'V-Tail'},
    {id: 7, label: 'V-Tail R'},
]
const CONTROL_FUNCTION_OPTIONS = [
    {id: 8, label: 'Gyro Mode'},
    {id: 9, label: 'Gyro Gain'},
]
const FUNCTION_OPTIONS = [...SURFACE_FUNCTION_OPTIONS, ...CONTROL_FUNCTION_OPTIONS]
const CONTROL_FUNCTION_IDS = new Set(CONTROL_FUNCTION_OPTIONS.map((option) => option.id))
const QUICK_SETUP_WING_OPTIONS = [
    {id: 0, label: 'Empty'},
    {id: 1, label: 'Normal'},
    {id: 2, label: '2-Aileron'},
    {id: 3, label: 'Delta'},
]
const QUICK_SETUP_TAIL_OPTIONS = [
    {id: 0, label: 'Empty'},
    {id: 1, label: 'Normal'},
    {id: 2, label: 'V-Tail'},
    {id: 3, label: 'Taileron'},
    {id: 4, label: 'Rudder Only'},
]
const CHANNEL_LIMIT_RANGES = {
    min: {min: 885, max: 1501},
    mid: {min: 1000, max: 2000},
    max: {min: 1501, max: 2135},
}

function channelLimitsAreValid(channelFunction) {
    return Object.entries(CHANNEL_LIMIT_RANGES).every(([key, limits]) =>
        Number.isFinite(channelFunction[key])
        && channelFunction[key] >= limits.min
        && channelFunction[key] <= limits.max)
        && channelFunction.min <= channelFunction.mid
        && channelFunction.mid <= channelFunction.max
}

function channelFunctionSupportsLimits(functionId) {
    return !CONTROL_FUNCTION_IDS.has(Number(functionId))
}

function formatChannelFunctionLimits(channelFunction) {
    return channelFunctionSupportsLimits(channelFunction.functionId) ? `${channelFunction.min} / ${channelFunction.mid} / ${channelFunction.max}` : ''
}

/**
 * Manages channel functions, limits, endpoint capture, and quick setup.
 *
 * `channelFunctions` is the dense 16-channel editor model returned by the receiver.
 * Channel-function saves are full replacements; config-file imports may instead send a
 * sparse list because the receiver clears every mapping before applying it.
 *
 * Emits `gyro-config-updated` after writes and `gyro-calibration-open` when
 * endpoint calibration must be owned by the top-level panel.
 */
@customElement('gyro-channel-functions-card')
export class GyroChannelFunctionsCard extends GyroElement {
    @property({attribute: false}) accessor channelFunctions = []
    @property({attribute: false}) accessor runtime = null
    @property({attribute: false}) accessor calibrationNeeded = false
    @property({attribute: false}) accessor calibrationBusy = false
    @state() accessor channelFunctionDialog = null
    @state() accessor channelFunctionSaving = false
    @state() accessor quickSetupDialog = null
    @state() accessor quickSetupSaving = false
    @state() accessor endpointCapture = null

    disconnectedCallback() {
        if (this.endpointCapture) postJSON('/gyro-calibration.json', {action: 'cancel'}, {
            keepalive: true,
            raw: true
        }).catch(() => {
        })
        super.disconnectedCallback()
    }


    _visibleChannelFunctions() {
        return this.channelFunctions.filter((row) => row.functionId !== 0)
    }

    _unusedChannelFunctions(currentChannel = null) {
        return this.channelFunctions.filter((row) => row.functionId === 0 || row.channel === currentChannel)
    }

    _unusedChannels(currentChannel) {
        return this._unusedChannelFunctions(currentChannel).map((row) => row.channel)
    }

    _isChannelFunctionConfigurationValid(draft) {
        const channelFunctions = this.channelFunctions.map((channelFunction) => channelFunction.channel === draft.channel
            ? {...channelFunction, functionId: Number(draft.functionId), master: Boolean(draft.master)}
            : channelFunction)
        const controls = channelFunctions.map((channelFunction) => Number(channelFunction.functionId))
            .filter((functionId) => CONTROL_FUNCTION_IDS.has(functionId))
        const masters = channelFunctions.filter((channelFunction) => channelFunctionSupportsLimits(channelFunction.functionId) && channelFunction.master)
            .map((channelFunction) => Number(channelFunction.functionId))
        return new Set(controls).size === controls.length
            && new Set(masters).size === masters.length
    }

    _availableChannelFunctions(draft) {
        return FUNCTION_OPTIONS.filter((option) => this._isChannelFunctionConfigurationValid(
            {...draft, functionId: option.id, master: option.id === draft.functionId ? draft.master : false},
        ))
    }

    _openChannelFunctionDialog(mode, channel = null) {
        const channelFunction = mode === 'add'
            ? this._unusedChannelFunctions()[0]
            : this.channelFunctions.find((row) => row.channel === channel)
        if (!channelFunction) return

        const draft = mode === 'add' ? {...channelFunction, functionId: 0} : {...channelFunction}
        if (mode === 'add') draft.functionId = this._availableChannelFunctions(draft)[0]?.id ?? 0
        this.channelFunctionDialog = {mode, draft}
        if (channelFunctionSupportsLimits(draft.functionId)) this._startEndpointCapture()
    }

    async _closeChannelFunctionDialog() {
        if (this.channelFunctionSaving) return
        await this._cancelEndpointCapture()
        this.channelFunctionDialog = null
    }

    _openQuickSetup() {
        this.quickSetupDialog = {step: 0, wing: 1, tail: 1}
    }

    _closeQuickSetup() {
        if (!this.quickSetupSaving) this.quickSetupDialog = null
    }

    _updateQuickSetup(key, value) {
        this.quickSetupDialog = {...this.quickSetupDialog, [key]: value}
    }

    async _submitQuickSetup(event) {
        event.preventDefault()
        if (this.quickSetupDialog.step === 0) {
            this.quickSetupDialog = {...this.quickSetupDialog, step: 1}
            return
        }
        this.quickSetupSaving = true
        try {
            await this.saveConfig({
                quick_setup: {
                    wing: Number(this.quickSetupDialog.wing),
                    tail: Number(this.quickSetupDialog.tail),
                }
            })
            this.quickSetupDialog = null
        } catch (error) {
            await showAlert('error', 'Quick Setup Failed', error?.message || 'Request failed')
        } finally {
            this.quickSetupSaving = false
        }
    }

    _updateChannelFunctionDraft(key, value) {
        if (this.channelFunctionDialog) {
            this.channelFunctionDialog = {
                ...this.channelFunctionDialog,
                draft: {...this.channelFunctionDialog.draft, [key]: value},
            }
        }
    }

    async _updateChannelFunction(functionId) {
        await this._cancelEndpointCapture()
        const hasMaster = this.channelFunctions.some((channelFunction) => channelFunction.channel !== this.channelFunctionDialog.draft.channel
            && Number(channelFunction.functionId) === functionId && channelFunction.master)
        this.channelFunctionDialog = {
            ...this.channelFunctionDialog,
            draft: {
                ...this.channelFunctionDialog.draft,
                functionId,
                master: channelFunctionSupportsLimits(functionId) && !hasMaster,
                invert: channelFunctionSupportsLimits(functionId) ? this.channelFunctionDialog.draft.invert : false,
            },
        }
        if (channelFunctionSupportsLimits(functionId)) this._startEndpointCapture()
    }

    async _updateChannel(channel) {
        await this._cancelEndpointCapture()
        this._updateChannelFunctionDraft('channel', channel)
        if (channelFunctionSupportsLimits(this.channelFunctionDialog.draft.functionId)) this._startEndpointCapture()
    }

    _stickLimits() {
        return this.runtime?.stick_limits?.[Number(this.channelFunctionDialog?.draft.channel) - 1] ?? null
    }

    async _startEndpointCapture() {
        this.endpointCapture = true
        try {
            await postJSON('/gyro-calibration.json', {action: 'sticks-range-start'})
        } catch (error) {
            this.endpointCapture = false
            await showAlert('error', 'Endpoint Capture Failed', error?.message || 'Request failed')
        }
    }

    async _cancelEndpointCapture() {
        if (!this.endpointCapture) return
        this.endpointCapture = false
        try {
            await postJSON('/gyro-calibration.json', {action: 'cancel'}, {raw: true})
        } catch (_error) {
        }
    }

    _copyStickRange() {
        const limits = this._stickLimits()
        const channel = Number(this.channelFunctionDialog?.draft.channel)
        const mid = Number(this.runtime?.channels?.[channel - 1])
        if (!limits || !Number.isFinite(Number(limits.min)) || !Number.isFinite(Number(limits.max)) || !Number.isFinite(mid)) return
        this._updateChannelFunctionDraft('min', Number(limits.min))
        this._updateChannelFunctionDraft('mid', mid)
        this._updateChannelFunctionDraft('max', Number(limits.max))
    }

    async _saveChannelFunctions(channelFunctions) {
        this.channelFunctionSaving = true
        try {
            const savedChannelFunctions = channelFunctions.map((channelFunction) => ({
                ...channelFunction,
                function: FUNCTION_OPTIONS.find((option) => option.id === Number(channelFunction.functionId))?.label ?? 'None',
            }))
            await this.saveConfig({channel_functions: savedChannelFunctions})
            this.channelFunctionDialog = null
        } catch (error) {
            await showAlert('error', 'Gyro Channel Function Save Failed', error?.message || 'Request failed')
        } finally {
            this.channelFunctionSaving = false
        }
    }

    async _submitChannelFunctionDialog(event) {
        event.preventDefault()
        if (!this.channelFunctionDialog) return

        const draft = {
            ...this.channelFunctionDialog.draft,
            channel: Number(this.channelFunctionDialog.draft.channel),
            functionId: Number(this.channelFunctionDialog.draft.functionId),
            min: Number(this.channelFunctionDialog.draft.min),
            mid: Number(this.channelFunctionDialog.draft.mid),
            max: Number(this.channelFunctionDialog.draft.max),
        }
        if (!channelFunctionSupportsLimits(draft.functionId)) {
            draft.master = false
            draft.invert = false
        }
        if (!this.channelFunctions.some((row) => row.channel === draft.channel)) {
            await showAlert('error', 'Invalid Channel', 'Choose a valid channel before saving.')
            return
        }
        if (!FUNCTION_OPTIONS.some((option) => option.id === draft.functionId)
            || !this._isChannelFunctionConfigurationValid(draft)) {
            await showAlert('error', 'Invalid Channel Function', 'Gyro Mode and Gyro Gain may each be mapped once; each surface function may have one master.')
            return
        }
        if (this.channelFunctionDialog.mode === 'add' && this.channelFunctions.some((row) => row.channel === draft.channel && row.functionId !== 0)) {
            await showAlert('error', 'Channel In Use', `CH${draft.channel} already has a mapping. Edit that row instead.`)
            return
        }
        if (channelFunctionSupportsLimits(draft.functionId) && !channelLimitsAreValid(draft)) {
            await showAlert('error', 'Invalid Channel Limits', 'Expected minimum 885–1501 µs, center 1000–2000 µs, maximum 1501–2135 µs, and min ≤ center ≤ max.')
            return
        }

        await this._cancelEndpointCapture()
        await this._saveChannelFunctions(
            this.channelFunctions.map((channelFunction) => channelFunction.channel === draft.channel ? draft : channelFunction),
        )
    }

    async _deleteChannelFunction(channel) {
        await this._saveChannelFunctions(this.channelFunctions.map((channelFunction) => channelFunction.channel === channel
            ? {...channelFunction, functionId: 0, master: false, invert: false}
            : channelFunction))
    }

    _openCalibration() {
        this.emit('gyro-calibration-open', {wizard: 'sticks'})
    }

    render() {
        const visibleChannelFunctions = this._visibleChannelFunctions()
        const channelFunctionDialog = this.channelFunctionDialog
        const channelFunctionDraft = channelFunctionDialog?.draft ?? null
        const channelFunctionHasOtherMaster = channelFunctionDraft && this.channelFunctions.some((channelFunction) => channelFunction.channel !== channelFunctionDraft.channel
            && Number(channelFunction.functionId) === Number(channelFunctionDraft.functionId) && channelFunction.master)
        const unusedChannels = this._unusedChannels(channelFunctionDraft?.channel)
        const unusedChannelFunction = this._unusedChannelFunctions()[0]
        const canAddChannelFunction = unusedChannelFunction && this._availableChannelFunctions(unusedChannelFunction).length > 0
        const availableChannelFunctions = channelFunctionDraft ? this._availableChannelFunctions(channelFunctionDraft) : []
        const endpointCapture = this.endpointCapture
        const stickLimits = this._stickLimits()

        return html`
            <div class="mui-panel gyro-card gyro-channel-functions-card">
                <div class="mui--text-title">Channel Functions &amp; Limits</div>
                <table class="mui-table">
                    <thead>
                    <tr>
                        <th>CH</th>
                        <th>Function</th>
                        <th>Master</th>
                        <th>Invert</th>
                        <th>Min / Mid / Max</th>
                        <th></th>
                    </tr>
                    </thead>
                    <tbody>
                    ${visibleChannelFunctions.length === 0 ? html`
                        <tr class="gyro-empty-row">
                            <td colspan="6">No active mappings. Add a mapping to assign a gyro channel function.</td>
                        </tr>
                    ` : visibleChannelFunctions.map((row) => html`
                        <tr>
                            <td>CH${row.channel}</td>
                            <td>${row.function}</td>
                            <td>${channelFunctionSupportsLimits(row.functionId) ? (row.master ? 'Yes' : 'No') : ''}</td>
                            <td>${channelFunctionSupportsLimits(row.functionId) ? (row.invert ? 'Yes' : 'No') : ''}</td>
                            <td>${formatChannelFunctionLimits(row)}</td>
                            <td>
                                <div class="gyro-row-actions">
                                    <button class="mui-btn mui-btn--small gyro-icon-button"
                                            ?disabled=${this.channelFunctionSaving}
                                            @click=${() => this._openChannelFunctionDialog('edit', row.channel)}>
                                        <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24">
                                            <path d="m3 17.25 9.81-9.81 3.75 3.75-9.81 9.81H3v-3.75ZM20.71 5.04a1 1 0 0 0 0-1.42l-1.33-1.33a1 1 0 0 0-1.42 0l-1.74 1.74 3.75 3.75 1.74-1.74Z"/>
                                        </svg>
                                    </button>
                                    <button class="mui-btn mui-btn--small gyro-icon-button"
                                            ?disabled=${this.channelFunctionSaving}
                                            @click=${() => this._deleteChannelFunction(row.channel)}>
                                        <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24">
                                            <path d="M9 3h6l1 2h4v2H4V5h4l1-2Zm-3 6h12l-1 12H7L6 9Zm4 2v8h2v-8h-2Zm4 0v8h2v-8h-2Z"/>
                                        </svg>
                                    </button>
                                </div>
                            </td>
                        </tr>
                    `)}
                    </tbody>
                </table>
                <div class="gyro-action-row gyro-channel-function-actions">
                    <div>
                        <button class="mui-btn mui-btn--small" type="button" ?disabled=${this.quickSetupSaving}
                                @click=${() => this._openQuickSetup()}>Quick Setup
                        </button>
                    </div>
                    <div>
                        <button class=${this.calibrationNeeded ? 'mui-btn mui-btn--small mui-btn--danger' : 'mui-btn mui-btn--small'}
                                ?disabled=${this.calibrationBusy} @click=${() => this._openCalibration()}>Endpoint Cal
                        </button>
                        <button class="mui-btn mui-btn--small" type="button"
                                ?disabled=${!canAddChannelFunction || this.channelFunctionSaving}
                                @click=${() => this._openChannelFunctionDialog('add')}>Add Channel
                        </button>
                    </div>
                </div>
                ${channelFunctionDialog && channelFunctionDraft ? html`
                    <elrs-wizard-dialog
                            .title=${channelFunctionDialog.mode === 'add' ? 'Add Mapping' : `Edit CH${channelFunctionDraft.channel}`}
                            icon="icon--symbols icon--symbols--gyro" ?closeDisabled=${this.channelFunctionSaving}
                            ?closeOnBackdrop=${true} @wizard-close=${() => this._closeChannelFunctionDialog()} .body=${html`
                        <form class="mui-form gyro-form" @submit=${(event) => this._submitChannelFunctionDialog(event)}>
                            <div class="mui-select">
                                <select id="gyro-channel-function-channel"
                                        ?disabled=${channelFunctionDialog.mode === 'edit' || this.channelFunctionSaving}
                                        @change=${(event) => this._updateChannel(Number(event.target.value))}>
                                    ${unusedChannels.map((channel) => html`
                                        <option value=${String(channel)} ?selected=${channel === channelFunctionDraft.channel}>
                                                CH${channel}
                                        </option>`)}
                                </select>
                                <label for="gyro-channel-function-channel">Channel</label>
                            </div>
                            <div class="mui-select">
                                <select id="gyro-channel-function-function" ?disabled=${this.channelFunctionSaving}
                                        @change=${(event) => this._updateChannelFunction(Number(event.target.value))}>
                                    <optgroup label="Control surfaces">
                                        ${availableChannelFunctions.filter((option) => option.id < 8).map((option) => html`
                                            <option value=${String(option.id)}
                                                    ?selected=${option.id === channelFunctionDraft.functionId}>${option.label}
                                            </option>`)}
                                    </optgroup>
                                    ${availableChannelFunctions.some((option) => option.id > 7) ? html`
                                        <optgroup label="Gyro controls">
                                            ${availableChannelFunctions.filter((option) => option.id > 7).map((option) => html`
                                                <option value=${String(option.id)}
                                                        ?selected=${option.id === channelFunctionDraft.functionId}>
                                                    ${option.label}
                                                </option>`)}
                                        </optgroup>` : ''}
                                </select>
                                <label for="gyro-channel-function-function">Function</label>
                            </div>
                            ${channelFunctionSupportsLimits(channelFunctionDraft.functionId) ? html`
                                <div class="gyro-grid gyro-grid--dialog">
                                    ${[['min', 'Min (µs)'], ['mid', 'Center (µs)'], ['max', 'Max (µs)']].map(([key, label]) => html`
                                        <div class="mui-textfield"><input id=${`gyro-channel-function-${key}`}
                                                                          class="gyro-compact-input" type="number"
                                                                          min=${String(CHANNEL_LIMIT_RANGES[key].min)}
                                                                          max=${String(CHANNEL_LIMIT_RANGES[key].max)}
                                                                          step="1" required
                                                                          .value=${String(channelFunctionDraft[key])}
                                                                          ?disabled=${this.channelFunctionSaving}
                                                                          @input=${(event) => this._updateChannelFunctionDraft(key, event.target.value === '' ? Number.NaN : Number(event.target.value))}><label
                                                for=${`gyro-channel-function-${key}`}>${label}</label></div>`)}
                                </div>
                                <div class="gyro-group-block">
                                    <div class="gyro-group-title">Stick Calibration · CH${channelFunctionDraft.channel}</div>
                                    <div class="gyro-note">Move CH${channelFunctionDraft.channel} through its full range, then
                                        center it and copy its endpoints, or enter the values directly above.
                                    </div>
                                    <gyro-stick-range
                                            .channel=${channelFunctionDraft.channel}
                                            .functionName=${FUNCTION_OPTIONS.find((option) => option.id === Number(channelFunctionDraft.functionId))?.label ?? ''}
                                            .limits=${stickLimits ?? {}}
                                            .input=${this.runtime?.channels?.[Number(channelFunctionDraft.channel) - 1] ?? 1500}
                                            .showEndpoints=${true}
                                            .showCopy=${true}
                                            .copyDisabled=${this.channelFunctionSaving || !endpointCapture || !stickLimits}
                                            @gyro-stick-range-copy=${() => this._copyStickRange()}>
                                    </gyro-stick-range>
                                </div>
                                <div class="mui-checkbox"><input id="gyro-channel-function-invert" type="checkbox"
                                                                 ?checked=${channelFunctionDraft.invert}
                                                                 ?disabled=${this.channelFunctionSaving}
                                                                 @change=${(event) => this._updateChannelFunctionDraft('invert', event.target.checked)}><label
                                        for="gyro-channel-function-invert">Invert direction</label></div>
                                <div class="mui-checkbox"><input id="gyro-channel-function-master" type="checkbox"
                                                                 ?checked=${channelFunctionDraft.master}
                                                                 ?disabled=${this.channelFunctionSaving || channelFunctionHasOtherMaster}
                                                                 @change=${(event) => this._updateChannelFunctionDraft('master', event.target.checked)}><label
                                        for="gyro-channel-function-master">Master channel</label></div>
                            ` : html`<p class="gyro-note">Gyro controls use their input directly; servo travel and
                                direction do not apply.</p>`}
                            <div class="gyro-save-row">
                                <button class="mui-btn mui-btn--primary mui-btn--small" type="submit"
                                        ?disabled=${this.channelFunctionSaving}>${this.channelFunctionSaving ? 'Saving…' : 'Save'}
                                </button>
                            </div>
                        </form>`}>
                    </elrs-wizard-dialog>
                ` : ''}
                ${this.quickSetupDialog ? html`
                    <elrs-wizard-dialog title="Quick Setup" icon="icon--symbols icon--symbols--gyro"
                                        ?closeDisabled=${this.quickSetupSaving} ?closeOnBackdrop=${true}
                                        @wizard-close=${() => this._closeQuickSetup()}
                                        .notice=${this.quickSetupDialog.step === 0 ? html`<strong>Warning</strong>
                                        <div>Quick setup replaces the existing gyro configuration.</div>` : ''}
                                        .noticeType=${'warning'} .body=${html`
                        <form class="mui-form gyro-form" @submit=${(event) => this._submitQuickSetup(event)}>
                            <div class="gyro-section-label">Step ${this.quickSetupDialog.step + 1} of 2</div>
                            ${this.quickSetupDialog.step === 0 ? html`<p>Choose the wing configuration.</p>
                            <div class="mui-select"><select id="gyro-quick-setup-wing"
                                                            ?disabled=${this.quickSetupSaving}
                                                            @change=${(event) => this._updateQuickSetup('wing', Number(event.target.value))}>${QUICK_SETUP_WING_OPTIONS.map((option) => html`
                                <option value=${String(option.id)}
                                        ?selected=${option.id === this.quickSetupDialog.wing}>${option.label}
                                </option>`)}</select><label for="gyro-quick-setup-wing">Wing type</label></div>` : html`
                                <p>Choose the tail configuration.</p>
                                <div class="mui-select"><select id="gyro-quick-setup-tail"
                                                                ?disabled=${this.quickSetupSaving}
                                                                @change=${(event) => this._updateQuickSetup('tail', Number(event.target.value))}>${QUICK_SETUP_TAIL_OPTIONS.map((option) => html`
                                    <option value=${String(option.id)}
                                            ?selected=${option.id === this.quickSetupDialog.tail}>${option.label}
                                    </option>`)}</select><label for="gyro-quick-setup-tail">Tail type</label></div>`}
                            <div class="gyro-save-row">
                                ${this.quickSetupDialog.step > 0 ? html`
                                    <button class="mui-btn mui-btn--small" type="button"
                                            ?disabled=${this.quickSetupSaving}
                                            @click=${() => this._updateQuickSetup('step', 0)}>Back
                                    </button>` : ''}
                                <button class="mui-btn mui-btn--primary mui-btn--small" type="submit"
                                        ?disabled=${this.quickSetupSaving}>
                                    ${this.quickSetupDialog.step === 0 ? 'Next' : this.quickSetupSaving ? 'Applying…' : 'Apply Quick Setup'}
                                </button>
                            </div>
                        </form>`}>
                    </elrs-wizard-dialog>
                ` : ''}
            </div>
        `
    }
}
