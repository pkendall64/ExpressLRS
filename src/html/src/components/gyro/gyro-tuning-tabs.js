import {html} from 'lit'
import {customElement, property, state} from 'lit/decorators.js'
import {GyroElement} from './gyro-element.js'
import './gyro-modes-card.js'
import './gyro-pids-card.js'

/** Presents mode tuning and advanced PID settings in MUICSS tabs. */
@customElement('gyro-tuning-tabs')
export class GyroTuningTabs extends GyroElement {
    @property({attribute: false}) accessor gyroModes = []
    @property({attribute: false}) accessor gyroPids = []
    @property({attribute: false}) accessor activeModeId = 0
    @state() accessor activeTab = 'modes'

    _selectTab(event, tab) {
        event.preventDefault()
        this.activeTab = tab
    }

    render() {
        const modesActive = this.activeTab === 'modes'
        return html`
            <div class="mui-panel gyro-card gyro-tuning-tabs">
                <ul class="mui-tabs__bar mui-tabs__bar--justified" role="tablist">
                    <li class=${modesActive ? 'mui--is-active' : ''} role="presentation">
                        <a href="#gyro-mode-tuning" role="tab" @click=${(event) => this._selectTab(event, 'modes')}>
                            Gyro Mode Tuning
                        </a>
                    </li>
                    <li class=${!modesActive ? 'mui--is-active' : ''} role="presentation">
                        <a href="#gyro-pid-tuning" role="tab" @click=${(event) => this._selectTab(event, 'pids')}>
                            Advanced (PIDs)
                        </a>
                    </li>
                </ul>
                <div id="gyro-mode-tuning" class=${`mui-tabs__pane ${modesActive ? 'mui--is-active' : ''}`}
                     role="tabpanel">
                    <gyro-modes-card .gyroModes=${this.gyroModes} .activeModeId=${this.activeModeId}></gyro-modes-card>
                </div>
                <div id="gyro-pid-tuning" class=${`mui-tabs__pane ${!modesActive ? 'mui--is-active' : ''}`}
                     role="tabpanel">
                    <gyro-pids-card .gyroPids=${this.gyroPids}></gyro-pids-card>
                </div>
            </div>
        `
    }
}
