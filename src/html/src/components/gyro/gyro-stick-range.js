import {html} from 'lit'
import {customElement, property} from 'lit/decorators.js'
import {GyroElement} from './gyro-element.js'

const RANGE_MIN_US = 885
const RANGE_MAX_US = 2135

/**
 * Renders one channel's center, endpoints, and live input on the ELRS pulse range.
 *
 * This component is presentation-only. When enabled, Copy emits
 * `gyro-stick-range-copy`; its owner decides how captured limits are persisted.
 */
@customElement('gyro-stick-range')
export class GyroStickRange extends GyroElement {
    @property({attribute: false}) accessor channel = 0
    @property({attribute: false}) accessor functionName = ''
    @property({attribute: false}) accessor limits = {}
    @property({attribute: false}) accessor input = 1500
    @property({attribute: false}) accessor showEndpoints = false
    @property({attribute: false}) accessor copyDisabled = false
    @property({attribute: false}) accessor showCopy = false

    _position(value) {
        const bounded = Math.min(RANGE_MAX_US, Math.max(RANGE_MIN_US, Number(value) || 1500))
        return `${(bounded - RANGE_MIN_US) * 100 / (RANGE_MAX_US - RANGE_MIN_US)}%`
    }

    render() {
        const {min = 1500, mid = 1500, max = 1500} = this.limits
        const minPosition = this._position(min)
        const maxPosition = this._position(max)
        return html`
            <div class="gyro-stick-range-card ${this.showCopy ? 'gyro-stick-range-card--copy' : ''}">
                <strong class="gyro-stick-channel">CH${this.channel}${this.functionName ? ` · ${this.functionName}` : ''}</strong>
                <div class="gyro-stick-range">
                    ${this.showEndpoints ? html`<span class="gyro-stick-whisker" style=${`left:${minPosition}; width:calc(${maxPosition} - ${minPosition});`}></span>` : ''}
                    <span class="gyro-stick-track"></span>
                    ${this.showEndpoints ? html`<span class="gyro-stick-marker gyro-stick-marker--endpoint" style=${`left:${minPosition};`} title=${`Min ${min} µs`}></span><span class="gyro-stick-marker gyro-stick-marker--endpoint" style=${`left:${maxPosition};`} title=${`Max ${max} µs`}></span>` : ''}
                    <span class="gyro-stick-marker gyro-stick-marker--center" style=${`left:${this._position(mid)};`} title=${`Center ${mid} µs`}></span>
                    <span class="gyro-stick-input-marker" style=${`left:${this._position(this.input)};`} title=${`Input ${this.input} µs`}></span>
                </div>
                <span class="gyro-stick-range-scale">${this.showEndpoints ? `${min} · ${mid} · ${max}` : `Center ${this.input} µs`}</span>
                ${this.showCopy ? html`<button class="mui-btn mui-btn--small" type="button" ?disabled=${this.copyDisabled} @click=${() => this.emit('gyro-stick-range-copy')}>Copy</button>` : ''}
            </div>
        `
    }
}
