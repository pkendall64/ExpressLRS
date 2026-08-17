import {LitElement} from 'lit'

/** Light-DOM base for gyro elements that share the group stylesheet. */
export class GyroElement extends LitElement {
    createRenderRoot() {
        return this
    }

    emit(type, detail) {
        this.dispatchEvent(new CustomEvent(type, {bubbles: true, composed: true, detail}))
    }

    async saveConfig(changes) {
        const response = await fetch('/gyro-config.json', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(changes),
        })
        if (!response.ok) throw new Error(await response.text() || 'Failed to save gyro config')
        this.emit('gyro-config-updated', changes)
    }
}
