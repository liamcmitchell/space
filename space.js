/// <reference types="@types/lodash" />
/// <reference types="gl-matrix" />
/// <reference types="twgl.js" />

const twgl = /** @type {import("twgl.js")} */ (window.twgl)
const { mat4, vec2 } = /** @type {import("gl-matrix")} */ (window.glMatrix)

function debugTimer(label, fn) {
  if (!location.hash.includes("debug")) return fn

  const debugLine = document.createElement("div")
  document.getElementById("debug").appendChild(debugLine)
  const pastWeight = 0.95
  let averageTime = 0

  return function () {
    const start = performance.now()
    const val = fn.apply(this, arguments)
    const time = performance.now() - start
    averageTime = (averageTime || time) * pastWeight + time * (1 - pastWeight)
    debugLine.innerHTML = `${label}${averageTime.toFixed(1)}ms`
    return val
  }
}

/**
 * @typedef {[number, number]} Vector
 */

/**
 * @typedef {object} Ship
 * @prop {number} mass
 * @prop {number} inertia
 * @prop {Vector} location
 * @prop {Vector} velocity
 * @prop {number} angle
 * @prop {number} angularVelocity
 * @prop {boolean} thrusting
 * @prop {boolean} landed
 * @prop {boolean} takingOff
 * @prop {boolean} destroyed
 * @prop {number} destroyedFor
 */

/**
 * @typedef {object} System
 * @prop {number} mass
 * @prop {number} orbitRadius
 * @prop {System[]} [children]
 */

/**
 * @typedef {object} SystemBody
 * @prop {number} mass
 * @prop {Vector} location
 * @prop {Vector} velocity
 */

/**
 * @typedef {object} State
 * @prop {boolean} running
 * @prop {boolean} rendered
 * @prop {number} time
 * @prop {number} timeInc
 * @prop {number} zoom
 * @prop {SystemBody[]} bodies
 * @prop {Ship} ship
 */

/** @type {Vector} */
const center = [0, 0]

/** @type {Vector[]} */
const shipBody = [
  [0, 0],
  [1, -1],
  [0, 2],
  [-1, -1],
  [0, 0],
]

const shipRadius = Math.max(
  ...shipBody.map((point) => vec2.distance(point, center))
)

/** @type {Vector[]} */
const shipThrust = [
  [-0.25, -0.25],
  [0, -1],
  [0.25, -0.25],
]

/** @type {State} */
const STATE = {
  running: document.hasFocus(),
}

function getTotalMass(bodies) {
  return bodies.reduce((memo, body) => {
    return memo + body.mass
  }, 0)
}

function getOrbitVelocity(parentMass, childMass, radius) {
  const force = getGravityForce(parentMass, childMass, radius)
  return Math.sqrt((force / childMass) * radius)
}

/**
 * @param {System} system
 * @param {number} time
 * @returns {SystemBody[]}
 */
function getSystemBodies(system, time) {
  const bodies = [
    {
      mass: system.mass,
      location: [0, 0],
      velocity: [0, 0],
    },
  ]
  if (system.children) {
    system.children.forEach((child) => {
      // Calculate child location
      const childBodies = getSystemBodies(child, time)
      if (!child.orbitVelocity) {
        child.orbitVelocity = getOrbitVelocity(
          system.mass,
          getTotalMass(childBodies),
          child.orbitRadius
        )
      }
      // velocity = distance / time
      // diameter = 2 * PI * radiusPx
      // rotation/time = velocity / diameter
      // angle/time = velocity / radius
      const childAngle = (child.orbitVelocity / child.orbitRadius) * time
      const childLocation = [
        Math.cos(childAngle) * child.orbitRadius,
        Math.sin(childAngle) * child.orbitRadius,
      ]
      const childVelocity = [
        Math.sin(childAngle) * -child.orbitVelocity,
        Math.cos(childAngle) * child.orbitVelocity,
      ]
      childBodies.forEach((body) => {
        vec2.add(body.location, body.location, childLocation)
        vec2.add(body.velocity, body.velocity, childVelocity)
        if (body.orbitCenter) {
          vec2.add(body.orbitCenter, body.orbitCenter, childLocation)
        } else {
          body.orbitCenter = [0, 0]
        }
        if (!body.orbitRadius) {
          body.orbitRadius = child.orbitRadius
        }
        bodies.push(body)
      })
    })
  }
  return bodies
}

/**
 * @param {SystemBody} body
 * @param {SystemBody[]} bodies
 * @returns {Vector}
 */
function getForceOn(body, bodies) {
  return bodies.reduce(
    (memo, otherBody) => {
      if (body === otherBody) {
        return memo
      }
      const force = getForceBetween(body, otherBody)
      return [memo[0] + force[0], memo[1] + force[1]]
    },
    [0, 0]
  )
}

/**
 * @param {number} angle
 * @param {number} distance
 * @returns {Vector}
 */
function getAngleVector(angle, distance) {
  return [Math.sin(angle) * distance, Math.cos(angle) * distance]
}

/**
 * @param {number} a
 * @param {number} b
 * @returns {number}
 */
function getAngleDifference(a, b) {
  const diff = (b - a) % (Math.PI * 2)
  if (diff > Math.PI) {
    return diff - Math.PI * 2
  } else if (diff < -Math.PI) {
    return diff + Math.PI * 2
  }
  return diff
}

/**
 * @param {number} massA
 * @param {number} massB
 * @returns {number}
 */
function getGravityForce(massA, massB, distance) {
  return (massA * massB) / Math.pow(distance, 2)
}

/**
 * @param {SystemBody} a
 * @param {SystemBody} b
 * @returns {Vector}
 */
function getForceBetween(a, b) {
  const out = vec2.create()
  vec2.subtract(out, b.location, a.location)
  const distance = vec2.length(out)
  const force = getGravityForce(a.mass, b.mass, distance)
  vec2.scale(out, out, force / distance)
  return out
}

function getBodyRadius(body) {
  return Math.pow(((body.mass / Math.PI) * 3) / 2, 1 / 3)
}

function _tick(force) {
  if (!STATE.running && !force) return

  STATE.rendered = false

  if (STATE.ship.destroyed && STATE.ship.destroyedFor > 2) {
    reset()
  }

  STATE.time += STATE.timeInc

  STATE.bodies = getSystemBodies(SYSTEM, STATE.time)

  // Handle keys
  if (KEYS["="]) {
    // Zoom in
    STATE.zoom = STATE.zoom * 1.05
  }
  if (KEYS["-"]) {
    // Zoom out
    STATE.zoom = STATE.zoom / 1.05
  }
  if (KEYS["ArrowUp"] && !STATE.ship.destroyed) {
    // Thrust
    STATE.ship.thrusting = true
    STATE.ship.landed = false
    vec2.add(
      STATE.ship.velocity,
      STATE.ship.velocity,
      getAngleVector(STATE.ship.angle, 0.1)
    )
  } else {
    STATE.ship.thrusting = false
  }
  if (KEYS["ArrowLeft"] && !STATE.ship.landed) {
    // Left
    STATE.ship.angularVelocity += 0.1
  }
  if (KEYS["ArrowRight"] && !STATE.ship.landed) {
    // Right
    STATE.ship.angularVelocity -= 0.1
  }

  if (STATE.ship.destroyed) {
    // Increment destroyed counter
    STATE.ship.destroyedFor += STATE.timeInc
    // Slowly decrease velocity (and player view)
    vec2.scale(STATE.ship.velocity, STATE.ship.velocity, 0.99)
  }

  if (!STATE.ship.destroyed) {
    // Apply gravity to ship
    vec2.scaleAndAdd(
      STATE.ship.velocity,
      STATE.ship.velocity,
      getForceOn(STATE.ship, STATE.bodies),
      STATE.timeInc / STATE.ship.mass
    )
  }

  // Move ship
  vec2.scaleAndAdd(
    STATE.ship.location,
    STATE.ship.location,
    STATE.ship.velocity,
    STATE.timeInc
  )
  STATE.ship.angle += STATE.ship.angularVelocity * STATE.timeInc
  STATE.ship.angle %= Math.PI * 2

  STATE.ship.colliding = false
  STATE.ship.landed = false

  if (!STATE.ship.destroyed) {
    for (const body of STATE.bodies) {
      const distance = vec2.distance(body.location, STATE.ship.location)
      const radius = getBodyRadius(body)

      if (distance < radius + shipRadius) {
        const transform = createTransform(
          STATE.ship.angle,
          STATE.ship.location,
          1
        )
        const points = shipBody.map((v) => vec2.transformMat4([], v, transform))
        /** @type {[Vector, number][]} */
        const contactPoints = []
        STATE.debugShapes = []
        for (let index = 0; index < points.length - 1; index++) {
          const a = points[index]
          const aDistance = vec2.distance(a, body.location)
          if (aDistance < radius) {
            contactPoints.push([a, aDistance])
          }
          const b = points[index + 1]
          const length = vec2.distance(a, b)
          const dot =
            ((body.location[0] - a[0]) * (b[0] - a[0]) +
              (body.location[1] - a[1]) * (b[1] - a[1])) /
            Math.pow(length, 2)
          const closest = [
            a[0] + dot * (b[0] - a[0]),
            a[1] + dot * (b[1] - a[1]),
          ]
          const closestDistance = vec2.distance(body.location, closest)
          if (
            closestDistance < radius &&
            vec2.distance(a, closest) + vec2.distance(closest, b) <
              length + 0.01
          ) {
            contactPoints.push([closest, closestDistance])
          }
        }

        // Process the closest points first, they have penetrated deepest.
        contactPoints.sort((a, b) => a[1] - b[1])

        for (const [point, pointBodyDistance] of contactPoints) {
          const shipPoint = vec2.subtract([], STATE.ship.location, point)
          const shipPointPerp = perpendicular(shipPoint)
          const shipPointAngularVelocity = vec2.scale(
            [],
            shipPointPerp,
            STATE.ship.angularVelocity
          )
          const pointRelativeVelocity = vec2.subtract(
            [],
            vec2.add([], STATE.ship.velocity, shipPointAngularVelocity),
            body.velocity
          )
          const bodyPoint = vec2.subtract([], point, body.location)
          const collisionNormal = vec2.normalize([], bodyPoint)
          const colliding = vec2.dot(collisionNormal, pointRelativeVelocity) < 0
          STATE.ship.colliding = STATE.ship.colliding || colliding
          if (colliding) {
            const impulseMagnitude =
              -vec2.dot(pointRelativeVelocity, collisionNormal) /
              (vec2.dot(
                collisionNormal,
                vec2.scale([], collisionNormal, 1 / STATE.ship.mass)
              ) +
                Math.pow(vec2.dot(shipPointPerp, collisionNormal), 2) /
                  STATE.ship.inertia)
            if (impulseMagnitude > 2) {
              STATE.ship.destroyed = true
              STATE.ship.destroyedFor = 0
              return
            }
            vec2.scaleAndAdd(
              STATE.ship.velocity,
              STATE.ship.velocity,
              collisionNormal,
              impulseMagnitude / STATE.ship.mass
            )
            STATE.ship.angularVelocity +=
              vec2.dot(
                shipPointPerp,
                vec2.scale([], collisionNormal, impulseMagnitude)
              ) / STATE.ship.inertia
            // Friction
            const collisionPerp = perpendicular(collisionNormal)
            const surfaceVelocity = vec2.dot(
              pointRelativeVelocity,
              collisionPerp
            )
            vec2.scaleAndAdd(
              STATE.ship.velocity,
              STATE.ship.velocity,
              collisionPerp,
              -surfaceVelocity / 20
            )
          }
          // Shift out of body.
          vec2.scaleAndAdd(
            STATE.ship.location,
            STATE.ship.location,
            collisionNormal,
            (radius - pointBodyDistance) / 2
          )
        }

        STATE.ship.landed =
          STATE.ship.colliding &&
          Math.abs(STATE.ship.angularVelocity) < 1 &&
          vec2.distance(STATE.ship.velocity, body.velocity) < 0.5 &&
          Math.abs(
            getAngleDifference(
              STATE.ship.angle,
              vec2.angle(STATE.ship.location, body.location)
            )
          ) < 0.5
      }
    }
  }
}

const tick = debugTimer("tick: ", _tick)

/**
 * @param {Vector} vector
 * @returns {Vector}
 */
function perpendicular(vector) {
  return [-vector[1], vector[0]]
}

/**
 * @typedef {object} Shape
 * @prop {'circle' | 'line'} type
 * @prop {mat4} transform
 * @prop {Vector[]} [points]
 * @prop {number[]} color
 * @prop {boolean} filled
 */

function worldShapes() {
  /** @type {Shape[]} */
  const shapes = []

  STATE.debugShapes?.forEach((shape) => {
    shapes.push(shape)
  })

  // Orbit rings
  STATE.bodies.forEach((body) => {
    if (body.orbitCenter) {
      shapes.push({
        type: "circle",
        transform: createTransform(0, body.orbitCenter, body.orbitRadius),
        color: [0.2, 0.2, 0.2, 1],
      })
    }
  })

  // Bodies
  STATE.bodies.forEach((body) => {
    shapes.push({
      type: "circle",
      transform: createTransform(0, body.location, getBodyRadius(body)),
      color: [1, 1, 1, 1],
      filled: true,
    })
  })

  // Ship
  const ship = STATE.ship
  if (ship.destroyed) {
    // Circle that gets bigger and transitions from yellow to red/transparent
    shapes.push({
      type: "circle",
      transform: createTransform(0, ship.location, 1 + 10 * ship.destroyedFor),
      color: [
        1,
        Math.max(0, 1 - ship.destroyedFor * 2),
        0,
        Math.max(0, 1 - ship.destroyedFor / 2),
      ],
    })
  } else {
    if (ship.thrusting) {
      // Yellow flame
      shapes.push({
        type: "lines",
        points: shipThrust,
        transform: createTransform(ship.angle, ship.location, 1),
        color: [1, 1, 0, 1],
        filled: true,
      })
    }
    // Ship body
    shapes.push({
      type: "lines",
      points: shipBody,
      transform: createTransform(ship.angle, ship.location, 1),
      color: [0.3, 0.5, 1, 1],
      filled: true,
    })
  }

  return shapes
}

function createTransform(rotation, translation, scale) {
  const transform = mat4.create()
  mat4.translate(transform, transform, [translation[0], translation[1], 0])
  mat4.rotateZ(transform, transform, -rotation)
  mat4.scale(transform, transform, [scale, scale, scale])
  return transform
}

function setupWebGlRender() {
  const canvas = document.createElement("canvas")
  document.body.appendChild(canvas)
  const gl = canvas.getContext("webgl")
  if (gl === null) {
    alert(
      "Unable to initialize WebGL. Your browser or machine may not support it."
    )
  }

  const vertexShader = `
    attribute vec4 position;
    uniform mat4 projection;
    uniform mat4 view;
    void main() {
      gl_Position = projection * view * position;
    }
  `

  const fragmentShader = `
    precision mediump float;
    uniform vec4 color;
    void main() {
      gl_FragColor = color;
    }
  `

  const programInfo = twgl.createProgramInfo(gl, [vertexShader, fragmentShader])

  function createCirclePositions(stops) {
    return _.times(stops * 2, (i) =>
      i % 2 === 0
        ? Math.cos((i * 2 * Math.PI) / (stops * 2))
        : Math.sin(((i - 1) * 2 * Math.PI) / (stops * 2))
    )
  }

  const bufferInfoMap = new WeakMap()

  function createBufferInfoFromArrays(data) {
    if (!bufferInfoMap.has(data)) {
      const flattenedData = typeof data[0] === "number" ? data : data.flat()
      bufferInfoMap.set(
        data,
        twgl.createBufferInfoFromArrays(gl, {
          position: { numComponents: 2, data: flattenedData },
        })
      )
    }
    return bufferInfoMap.get(data)
  }

  const bufferInfos = {
    circle: createBufferInfoFromArrays(createCirclePositions(100)),
    shipBody: createBufferInfoFromArrays(shipBody),
    shipThrust: createBufferInfoFromArrays(shipThrust),
  }

  const projection = mat4.create()

  let lastBufferInfo
  function setBuffersAndAttributes(bufferInfo) {
    if (lastBufferInfo !== bufferInfo) {
      twgl.setBuffersAndAttributes(gl, programInfo, bufferInfo)
      lastBufferInfo = bufferInfo
    }
  }

  /**
   * @param {Shape[]} shapes
   */
  return function renderWebGl(shapes) {
    const width = canvas.clientWidth * devicePixelRatio
    const height = canvas.clientHeight * devicePixelRatio

    twgl.resizeCanvasToDisplaySize(canvas, devicePixelRatio)
    gl.viewport(0, 0, width, height)

    gl.enable(gl.DEPTH_TEST)
    gl.depthFunc(gl.LEQUAL)

    gl.enable(gl.BLEND)
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)

    gl.clearColor(0.0, 0.0, 0.0, 1.0)
    gl.clearDepth(1.0)
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

    gl.useProgram(programInfo.program)

    mat4.identity(projection)
    mat4.scale(projection, projection, [
      2 / (width / devicePixelRatio),
      -2 / (height / devicePixelRatio),
      1,
    ])
    mat4.scale(projection, projection, [STATE.zoom, STATE.zoom, STATE.zoom])
    mat4.translate(projection, projection, [
      -STATE.ship.location[0],
      -STATE.ship.location[1],
      0,
    ])

    twgl.setUniforms(programInfo, {
      projection,
    })

    shapes.forEach((shape) => {
      twgl.setUniforms(programInfo, {
        view: shape.transform,
        color: shape.color,
      })
      if (shape.type == "circle") {
        setBuffersAndAttributes(bufferInfos.circle)
        twgl.drawBufferInfo(
          gl,
          bufferInfos.circle,
          shape.filled ? gl.TRIANGLE_FAN : gl.LINE_LOOP
        )
      }
      if (shape.type == "lines") {
        const bufferInfo = createBufferInfoFromArrays(shape.points)
        setBuffersAndAttributes(bufferInfo)
        twgl.drawBufferInfo(
          gl,
          bufferInfo,
          shape.filled ? gl.TRIANGLE_FAN : gl.LINE_STRIP
        )
      }
    })
  }
}

const renderShapes = debugTimer("render: ", setupWebGlRender())

function render() {
  if (!STATE.rendered) {
    renderShapes(worldShapes())
    STATE.rendered = true
  }
  requestAnimationFrame(render)
}

// Set event listeners
const KEYS = {}
document.addEventListener("keydown", (event) => {
  KEYS[event.key] = true
  if (event.key === "r") {
    reset()
    tick(true)
  }
})
document.addEventListener("keyup", (event) => {
  KEYS[event.key] = false
})
window.addEventListener("focus", () => {
  STATE.running = true
})
window.addEventListener("blur", () => {
  STATE.running = false
})
window.addEventListener("resize", () => {
  STATE.rendered = false
  requestAnimationFrame(render)
})

/** @type {System} */
const SYSTEM = {
  mass: 10000,
  orbitRadius: 0,
  children: [
    {
      mass: 20,
      orbitRadius: 50,
    },
    {
      mass: 30,
      orbitRadius: 100,
    },
    {
      mass: 50,
      orbitRadius: 200,
      children: [
        {
          mass: 20,
          orbitRadius: 10,
        },
      ],
    },
    {
      mass: 200,
      orbitRadius: 400,
      children: [
        {
          mass: 20,
          orbitRadius: 20,
        },
        {
          mass: 10,
          orbitRadius: 50,
        },
      ],
    },
  ],
}

function reset() {
  STATE.rendered = false
  STATE.time = 0
  STATE.timeInc = 1 / 100
  STATE.zoom = 10
  STATE.bodies = getSystemBodies(SYSTEM, STATE.time)
  const bodyToStartOn = STATE.bodies[5]
  STATE.ship = {
    mass: 1,
    inertia: 1,
    location: vec2.add([], bodyToStartOn.location, [
      0,
      -getBodyRadius(bodyToStartOn) - shipRadius + 1,
    ]),
    velocity: bodyToStartOn.velocity,
    angle: Math.PI,
    angularVelocity: 0,
    thrusting: false,
    landed: false,
  }
}

reset()

setInterval(tick, 16)
tick(true)
render()
