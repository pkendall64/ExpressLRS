// FEATURE:NOT IS_TX
// FEATURE:NOT IS_8285
import gyroCss from '../assets/gyro.css?inline'
import '../pages/gyro-panel.js'

// Gyro components use light DOM, so install their feature stylesheet once when
// this lazy-loaded group is requested instead of rendering a style tag per component.

const style = document.createElement('style')
style.textContent = gyroCss
document.head.append(style)
// /FEATURE:NOT IS_8285
// /FEATURE:NOT IS_TX
