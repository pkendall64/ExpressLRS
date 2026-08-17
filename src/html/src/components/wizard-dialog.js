import {html, LitElement} from 'lit'
import {customElement, property} from 'lit/decorators.js'
import {addEscapeHandler} from '../utils/overlay.js'

@customElement('elrs-wizard-dialog')
export class ElrsWizardDialog extends LitElement {
    @property() accessor title = ''
    @property() accessor icon = ''
    @property({type: Boolean}) accessor closeDisabled = false
    @property({type: Boolean}) accessor closeOnBackdrop = false
    @property({attribute: false}) accessor body = html``
    @property({attribute: false}) accessor notice = ''
    @property() accessor noticeType = 'error'
    removeEscapeHandler = null

    connectedCallback() {
        super.connectedCallback()
        this.removeEscapeHandler = addEscapeHandler(() => this._close())
    }

    disconnectedCallback() {
        this.removeEscapeHandler?.()
        this.removeEscapeHandler = null
        super.disconnectedCallback()
    }


    createRenderRoot() {
        return this
    }

    _close() {
        if (!this.closeDisabled) {
            this.dispatchEvent(new CustomEvent('wizard-close', {bubbles: true, composed: true}))
        }
    }

    _closeFromBackdrop(event) {
        if (this.closeOnBackdrop && event.target === event.currentTarget) {
            this._close()
        }
    }

    render() {
        return html`
            <div class="alert-wrapper wizard-dialog-backdrop" @click=${this._closeFromBackdrop}>
                <div class="alert-frame wizard">
                    <div class="alert-header">
                        <div class="alert-title-row">
                            <span class="alert-title-icon ${this.icon}" aria-hidden="true"></span>
                            <div class="wizard-title mui--text-title">${this.title}</div>
                        </div>
                        <span class="alert-close" @click=${() => this._close()}>X</span>
                    </div>
                    <div class="alert-body">
                        ${this.notice ? html`<div class="mui-panel ${this.noticeType}-bg">${this.notice}</div>` : ''}
                        ${this.body}
                    </div>
                </div>
            </div>
        `
    }
}
