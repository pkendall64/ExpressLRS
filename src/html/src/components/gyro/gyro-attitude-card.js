import {html, nothing} from 'lit'
import {customElement, property, query, state} from 'lit/decorators.js'
import {GyroElement} from './gyro-element.js'
import {createGyroOglView} from './gyro-ogl-view.js'
import smallAirplaneGlbUrl from '../../assets/small-airplane.glb?url'

function normalizeDegrees(angle) {
    let normalized = Number(angle) || 0
    while (normalized > 180) normalized -= 360
    while (normalized <= -180) normalized += 360
    return normalized
}

/**
 * Presents live receiver attitude as WebGL, horizon, compass, and numeric overlays.
 *
 * Reset Heading is display-local; it never writes receiver configuration.
 * The OGL view owns its animation loop and is destroyed with the component.
 */
@customElement('gyro-attitude-card')
export class GyroAttitudeCard extends GyroElement {
    @property({attribute: false}) accessor runtime = null
    @query('.gyro-ogl-holder') accessor holder
    @state() accessor headingOffset = 0
    @state() accessor floating = true
    @state() accessor minimized = true
    @state() accessor error = ''
    oglView = null
    // Shared by Lit rendering and the OGL animation callback to avoid allocating
    // a new Euler object on every animation frame.
    euler = {roll: 0, pitch: 0, yaw: 0}

    _euler() {
        this.euler.roll = -(Number(this.runtime?.angle_r) || 0)
        this.euler.pitch = Number(this.runtime?.angle_p) || 0
        this.euler.yaw = normalizeDegrees((Number(this.runtime?.angle_y) || 0) - this.headingOffset)
        return this.euler
    }

    async firstUpdated() {
        try {
            this.oglView = await createGyroOglView(this.holder, smallAirplaneGlbUrl, () => this._euler())
        } catch (error) {
            this.error = error?.message || 'Failed to start OGL attitude view'
        }
    }

    disconnectedCallback() {
        this.oglView?.destroy()
        this.oglView = null
        super.disconnectedCallback()
    }

    _resetHeading() {
        this.headingOffset = Number(this.runtime?.angle_y) || 0
    }

    _toggleFloat() {
        this.floating = !this.floating
        this.minimized = false
    }

    render() {
        const euler = this._euler()
        const horizonPitch = Math.max(-60, Math.min(60, euler.pitch))
        return html`
            <div class="mui-panel gyro-card gyro-attitude-card ${this.floating ? 'gyro-attitude-card--floating' : ''} ${this.minimized ? 'gyro-attitude-card--minimized' : ''}">
                <div class="gyro-card-title-row">
                    <div class="mui--text-title">Live Attitude View</div>
                    <button class="mui-btn mui-btn--small gyro-attitude-float-toggle" type="button"
                            @click=${() => this.floating ? this.minimized = true : this._toggleFloat()}>
                        ${this.floating ? 'Minimize' : 'Float'}
                    </button>
                </div>
                <div class="mui-panel gyro-ogl-stage">
                    <div class="gyro-ogl-holder"></div>
                    <div class="gyro-horizon" role="img"
                         aria-label=${`Artificial horizon: roll ${euler.roll.toFixed(1)} degrees, pitch ${euler.pitch.toFixed(1)} degrees`}>
                        <div class="gyro-horizon-world"
                             style=${`transform: rotate(${euler.roll}deg) translateY(${horizonPitch * 0.8}px)`}>
                            <div class="gyro-horizon-sky"></div>
                            <div class="gyro-horizon-ground"></div>
                        </div>
                        <div class="gyro-horizon-reference"></div>
                    </div>
                    <div class="gyro-compass" role="img"
                         aria-label=${`Compass heading ${euler.yaw.toFixed(1)} degrees`}>
                        <div class="gyro-compass-disc" style=${`transform: rotate(${-euler.yaw}deg)`}><span
                                class="gyro-compass-north">N</span><span class="gyro-compass-east">E</span><span
                                class="gyro-compass-south">S</span><span class="gyro-compass-west">W</span>
                        </div>
                        <div class="gyro-compass-pointer">▲</div>
                    </div>
                    <div class="gyro-euler-overlay">
                        <div>Roll ${euler.roll >= 0 ? '+' : ''}${euler.roll.toFixed(3)}°</div>
                        <div>Pitch ${euler.pitch >= 0 ? '+' : ''}${euler.pitch.toFixed(3)}°</div>
                        <div>Yaw ${euler.yaw >= 0 ? '+' : ''}${euler.yaw.toFixed(3)}°</div>
                    </div>
                </div>
                ${this.error ? html`<p class="gyro-note">${this.error}</p>` : nothing}
                <div class="gyro-action-row">
                    <button class="mui-btn mui-btn--small" @click=${() => this._resetHeading()}>Reset Heading</button>
                </div>
            </div>
            ${this.minimized ? html`
                <button class="mui-btn mui-btn--primary mui-btn--fab mui--z2 gyro-attitude-float-button" type="button"
                        aria-label="Restore Live Attitude" title="Restore Live Attitude" @click=${() => {
                    this.floating = true;
                    this.minimized = false
                }}>+
                </button>` : nothing}
        `
    }
}
