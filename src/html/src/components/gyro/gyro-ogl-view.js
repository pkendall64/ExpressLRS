import {Renderer, Camera, Transform, Program, Geometry, Mesh, GLTFLoader, Vec3} from 'ogl'

function measure(host) {
    return {
        width: Math.max(280, Math.floor(host.clientWidth || 420)),
        height: Math.max(220, Math.floor(host.clientHeight || 312)),
    }
}

function createProgram(gl, material = {}) {
    const baseColorFactor = material.baseColorFactor || [1, 1, 1, 1]
    return new Program(gl, {
        vertex: /* glsl */ `
            precision highp float;
            attribute vec3 position;
            attribute vec3 normal;
            uniform mat3 normalMatrix;
            uniform mat4 modelViewMatrix;
            uniform mat4 projectionMatrix;
            varying vec3 vNormal;
            void main() {
                vNormal = normalize(normalMatrix * normal);
                gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
            }
        `,
        fragment: /* glsl */ `
            precision highp float;
            varying vec3 vNormal;
            uniform vec4 uBaseColorFactor;
            uniform vec3 uLightDir;
            void main() {
                vec3 n = normalize(vNormal);
                float diffuse = max(dot(n, normalize(uLightDir)), 0.0);
                float light = 0.72 + diffuse * 0.38;
                vec3 albedo = mix(uBaseColorFactor.rgb, vec3(1.0), 0.18);
                gl_FragColor = vec4(albedo * light, uBaseColorFactor.a);
            }
        `,
        uniforms: {
            uBaseColorFactor: {value: baseColorFactor},
            uLightDir: {value: [0.25, 0.95, 0.45]},
        },
        cullFace: false,
    })
}

function createGroundGrid(gl, radius) {
    const extent = radius * 5
    const spacing = extent / 8
    const y = -radius * 0.6
    const vertices = []
    for (let step = -8; step <= 8; step++) {
        const offset = step * spacing
        vertices.push(offset, y, -extent, offset, y, extent)
        vertices.push(-extent, y, offset, extent, y, offset)
    }

    const program = new Program(gl, {
        vertex: /* glsl */ `
            precision highp float;
            attribute vec3 position;
            uniform mat4 modelViewMatrix;
            uniform mat4 projectionMatrix;
            void main() {
                gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
            }
        `,
        fragment: /* glsl */ `
            precision highp float;
            void main() {
                gl_FragColor = vec4(0.67, 0.78, 0.87, 1.0);
            }
        `,
        depthTest: true,
    })

    return new Mesh(gl, {
        geometry: new Geometry(gl, {position: {size: 3, data: new Float32Array(vertices)}}),
        mode: gl.LINES,
        program,
    })
}

/**
 * Creates and runs the attitude card's OGL scene.
 *
 * Model bounds determine camera distance and ground-grid scale. Rendering is
 * skipped while the host is hidden, but the animation loop remains scheduled so
 * a minimized view resumes without lifecycle coordination.
 *
 * @param {HTMLElement} host Container that owns the generated canvas.
 * @param {string} modelUrl URL of the aircraft GLB model.
 * @param {() => {roll: number, pitch: number, yaw: number}} getEuler Current display attitude in degrees.
 * @returns {Promise<{destroy: () => void}>} Cleanup handle for the frame loop, observer, and canvas.
 */
export async function createGyroOglView(host, modelUrl, getEuler) {
    const renderer = new Renderer({dpr: Math.min(window.devicePixelRatio || 1, 2), alpha: true, antialias: true})
    const gl = renderer.gl
    host.appendChild(gl.canvas)
    gl.clearColor(0.82, 0.90, 0.97, 1)

    const camera = new Camera(gl, {near: 0.1, far: 100})
    const scene = new Transform()
    const tiltGroup = new Transform()
    tiltGroup.setParent(scene)

    const resize = () => {
        const {width, height} = measure(host)
        renderer.setSize(width, height)
        camera.perspective({aspect: gl.canvas.width / gl.canvas.height})
    }
    resize()

    const gltf = await GLTFLoader.load(gl, modelUrl)
    const roots = gltf.scene || gltf.scenes[0] || []
    roots.forEach((root) => {
        root.setParent(tiltGroup)
        root.traverse((node) => {
            if (node.program) {
                node.program = createProgram(gl, node.program.gltfMaterial)
            }
        })
    })

    scene.updateMatrixWorld()
    const min = new Vec3(+Infinity, +Infinity, +Infinity)
    const max = new Vec3(-Infinity, -Infinity, -Infinity)
    const center = new Vec3()
    const scale = new Vec3()
    const boundsMin = new Vec3()
    const boundsMax = new Vec3()
    const boundsCenter = new Vec3()
    const boundsScale = new Vec3()

    gltf.meshes.forEach((group) => {
        group.primitives.forEach((mesh) => {
            if (!mesh.parent) return
            if (!mesh.geometry.bounds) mesh.geometry.computeBoundingSphere()
            boundsCenter.copy(mesh.geometry.bounds.center).applyMatrix4(mesh.worldMatrix)
            mesh.worldMatrix.getScaling(boundsScale)
            const radiusScale = Math.max(boundsScale[0], boundsScale[1], boundsScale[2])
            const radius = mesh.geometry.bounds.radius * radiusScale
            boundsMin.set(-radius, -radius, -radius).add(boundsCenter)
            boundsMax.set(radius, radius, radius).add(boundsCenter)
            for (let i = 0; i < 3; i++) {
                min[i] = Math.min(min[i], boundsMin[i])
                max[i] = Math.max(max[i], boundsMax[i])
            }
        })
    })

    scale.sub(max, min)
    center.add(min, max).divide(2)
    tiltGroup.position.sub(center)

    const maxRadius = Math.max(scale[0], scale[1], scale[2]) * 0.5 || 1
    createGroundGrid(gl, maxRadius).setParent(scene)
    camera.position.set(0, maxRadius * 0.14, maxRadius * 1.96)
    camera.lookAt(new Vec3(0, maxRadius * 0.04, 0))

    let frame = 0
    const update = () => {
        frame = requestAnimationFrame(update)
        if (host.offsetParent === null) return
        const {roll = 0, pitch = 0, yaw = 0} = getEuler() || {}
        tiltGroup.rotation.x = pitch * Math.PI / 180
        tiltGroup.rotation.y = -(yaw * Math.PI / 180)
        tiltGroup.rotation.z = (roll - 1.5) * Math.PI / 180
        renderer.render({scene, camera})
    }
    update()

    const resizeObserver = new ResizeObserver(() => resize())
    resizeObserver.observe(host)

    return {
        destroy() {
            cancelAnimationFrame(frame)
            resizeObserver.disconnect()
            gl.canvas.remove()
        }
    }
}
