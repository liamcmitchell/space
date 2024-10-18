/// <reference types="@types/lodash" />
/// <reference types="gl-matrix" />
/// <reference types="twgl.js" />

const twgl = /** @type {import("twgl.js")} */ (window.twgl)
const { mat4, quat, vec2 } = /** @type {import("gl-matrix")} */ (
  window.glMatrix
)

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
  ...shipBody.map((point) => getDistance(point, center))
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
        body.location = translate(body.location, childLocation)
        body.velocity = translate(body.velocity, childVelocity)
        if (body.orbitCenter) {
          body.orbitCenter = translate(body.orbitCenter, childLocation)
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
 * @param {Vector} a
 * @param {Vector} b
 * @returns {number}
 */
function getDistance(a, b) {
  return Math.sqrt(
    Math.pow(getDifferenceX(a, b), 2) + Math.pow(getDifferenceY(a, b), 2)
  )
}

/**
 * @param {Vector} v
 * @returns {number}
 */
function getMagnitude(v) {
  return Math.sqrt(Math.pow(v[0], 2) + Math.pow(v[1], 2))
}

/**
 * @param {Vector} a
 * @param {Vector} b
 * @returns {Vector}
 */
function getDifference(a, b) {
  return [b[0] - a[0], b[1] - a[1]]
}

/**
 * @param {Vector} a
 * @param {Vector} b
 * @returns {number}
 */
function getDifferenceX(a, b) {
  return b[0] - a[0]
}

/**
 * @param {Vector} a
 * @param {Vector} b
 * @returns {number}
 */
function getDifferenceY(a, b) {
  return b[1] - a[1]
}

/**
 * @param {Vector} a
 * @param {Vector} b
 * @returns {number}
 */
function getAngle(a, b) {
  return Math.atan2(a[0] - b[0], a[1] - b[1])
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
  const distance = getDistance(a.location, b.location)
  const force = getGravityForce(a.mass, b.mass, distance)
  return [
    (force * getDifferenceX(a.location, b.location)) / distance,
    (force * getDifferenceY(a.location, b.location)) / distance,
  ]
}

function getBodyRadius(body) {
  return Math.pow(((body.mass / Math.PI) * 3) / 2, 1 / 3)
}

function tick(force) {
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
    STATE.ship.velocity = translate(
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
    STATE.ship.velocity = scale(STATE.ship.velocity, 0.99)
  }

  if (!STATE.ship.destroyed) {
    // Apply gravity to ship
    STATE.ship.velocity = translate(
      STATE.ship.velocity,
      scale(
        getForceOn(STATE.ship, STATE.bodies),
        STATE.timeInc / STATE.ship.mass
      )
    )
  }

  // Move ship
  STATE.ship.location = translate(
    STATE.ship.location,
    scale(STATE.ship.velocity, STATE.timeInc)
  )
  STATE.ship.angle += STATE.ship.angularVelocity * STATE.timeInc
  STATE.ship.angle %= Math.PI * 2

  STATE.ship.colliding = false
  STATE.ship.landed = false

  if (!STATE.ship.destroyed) {
    for (const body of STATE.bodies) {
      const distance = getDistance(body.location, STATE.ship.location)
      const radius = getBodyRadius(body)

      if (distance < radius + shipRadius) {
        const points = shipBody.map((v) =>
          translate(rotate(v, center, STATE.ship.angle), STATE.ship.location)
        )
        /** @type {[Vector, number][]} */
        const contactPoints = []
        STATE.debugShapes = []
        for (let index = 0; index < points.length - 1; index++) {
          const a = points[index]
          const aDistance = getDistance(a, body.location)
          if (aDistance < radius) {
            contactPoints.push([a, aDistance])
          }
          const b = points[index + 1]
          const length = getDistance(a, b)
          const dot =
            ((body.location[0] - a[0]) * (b[0] - a[0]) +
              (body.location[1] - a[1]) * (b[1] - a[1])) /
            Math.pow(length, 2)
          const closest = [
            a[0] + dot * (b[0] - a[0]),
            a[1] + dot * (b[1] - a[1]),
          ]
          const closestDistance = getDistance(body.location, closest)
          if (
            closestDistance < radius &&
            getDistance(a, closest) + getDistance(closest, b) < length + 0.01
          ) {
            contactPoints.push([closest, closestDistance])
          }
        }

        // Process the closest points first, they have penetrated deepest.
        contactPoints.sort((a, b) => a[1] - b[1])

        for (const [point, pointBodyDistance] of contactPoints) {
          const shipPoint = getDifference(point, STATE.ship.location)
          const shipPointPerp = perpendicular(shipPoint)
          const shipPointAngularVelocity = scale(
            shipPointPerp,
            STATE.ship.angularVelocity
          )
          const pointRelativeVelocity = getDifference(
            body.velocity,
            translate(STATE.ship.velocity, shipPointAngularVelocity)
          )
          const bodyPoint = getDifference(body.location, point)
          const collisionNormal = normalize(bodyPoint)
          const colliding = dot(collisionNormal, pointRelativeVelocity) < 0
          STATE.ship.colliding = STATE.ship.colliding || colliding
          if (colliding) {
            const impulseMagnitude =
              -dot(scale(pointRelativeVelocity, 1), collisionNormal) /
              (dot(
                collisionNormal,
                scale(collisionNormal, 1 / STATE.ship.mass)
              ) +
                Math.pow(dot(shipPointPerp, collisionNormal), 2) /
                  STATE.ship.inertia)
            if (impulseMagnitude > 2) {
              STATE.ship.destroyed = true
              STATE.ship.destroyedFor = 0
              return
            }
            STATE.ship.velocity = translate(
              STATE.ship.velocity,
              scale(collisionNormal, impulseMagnitude / STATE.ship.mass)
            )
            STATE.ship.angularVelocity +=
              dot(shipPointPerp, scale(collisionNormal, impulseMagnitude)) /
              STATE.ship.inertia
            // Friction
            const collisionPerp = perpendicular(collisionNormal)
            const surfaceVelocity = dot(pointRelativeVelocity, collisionPerp)
            STATE.ship.velocity = translate(
              STATE.ship.velocity,
              scale(collisionPerp, -surfaceVelocity / 20)
            )
          }
          // Shift out of body.
          STATE.ship.location = translate(
            STATE.ship.location,
            scale(collisionNormal, (radius - pointBodyDistance) / 2)
          )
        }

        STATE.ship.landed =
          STATE.ship.colliding &&
          Math.abs(STATE.ship.angularVelocity) < 1 &&
          getDistance(STATE.ship.velocity, body.velocity) < 0.5 &&
          Math.abs(
            getAngleDifference(
              STATE.ship.angle,
              getAngle(STATE.ship.location, body.location)
            )
          ) < 0.5
      }
    }
  }
}

/**
 * @param {Vector} xy
 * @param {Vector} by
 * @returns {Vector}
 */
function translate(xy, by) {
  return [xy[0] + by[0], xy[1] + by[1]]
}

/**
 * @param {Vector} v
 * @returns {Vector}
 */
function normalize(v) {
  const size = getMagnitude(v)
  if (size === 0) return v
  return [v[0] / size, v[1] / size]
}

/**
 * @param {Vector} a
 * @param {Vector} b
 * @returns {number}
 */
function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1]
}

/**
 * @param {Vector} xy
 * @param {number} factor
 * @returns {Vector}
 */
function scale(xy, factor) {
  return [xy[0] * factor, xy[1] * factor]
}

/**
 * @param {Vector} vector
 * @returns {Vector}
 */
function perpendicular(vector) {
  return [-vector[1], vector[0]]
}

/**
 * @param {Vector} xy
 * @param {Vector} about
 * @param {number} angle
 * @returns {Vector}
 */
function rotate(xy, about, angle) {
  const distance = getDistance(xy, about)
  const currentAngle = getAngle(xy, about)
  return [
    about[0] + Math.sin(currentAngle + angle) * distance,
    about[1] + Math.cos(currentAngle + angle) * distance,
  ]
}

/**
 * @typedef {object} Shape
 * @prop {'circle' | 'line'} type
 * @prop {mat4} transform
 * @prop {Vector[]} [points]
 * @prop {string} strokeStyle
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
      })
    }
    // Ship body
    shapes.push({
      type: "lines",
      points: shipBody,
      transform: createTransform(ship.angle, ship.location, 1),
      color: ship.landed
        ? [0.1, 1, 0.1, 1]
        : ship.colliding
        ? [1, 0.2, 0.2, 1]
        : [0.3, 0.5, 1, 1],
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

    let lastBuffer
    shapes.forEach((shape) => {
      twgl.setUniforms(programInfo, {
        view: shape.transform,
        color: parseColor(shape.color),
      })
      if (shape.type == "circle") {
        const bufferInfo = bufferInfos.circle
        if (lastBuffer !== bufferInfo) {
          twgl.setBuffersAndAttributes(gl, programInfo, bufferInfo)
          lastBuffer = bufferInfo
        }
        twgl.drawBufferInfo(gl, bufferInfos.circle, gl.LINE_LOOP)
      }
      if (shape.type == "lines") {
        const bufferInfo = createBufferInfoFromArrays(shape.points)
        if (lastBuffer !== bufferInfo) {
          twgl.setBuffersAndAttributes(gl, programInfo, bufferInfo)
          lastBuffer = bufferInfo
        }
        twgl.drawBufferInfo(gl, bufferInfo, gl.LINE_STRIP)
      }
    })
  }
}

function setup2dRender() {
  const canvas = document.createElement("canvas")
  document.body.appendChild(canvas)
  const ctx = canvas.getContext("2d")

  const projection = mat4.create()
  const view = mat4.create()
  const center = vec2.create()
  const edge = vec2.fromValues(1, 0)
  const v1 = vec2.create()
  const v2 = vec2.create()

  /**
   * @param {Shape[]} shapes
   */
  return function render2d(shapes) {
    const width = canvas.clientWidth * devicePixelRatio
    const height = canvas.clientHeight * devicePixelRatio
    canvas.width = width
    canvas.height = height
    ctx.fillRect(0, 0, width, height)

    mat4.identity(projection)
    mat4.translate(projection, projection, [
      (canvas.clientWidth / 2) * devicePixelRatio,
      (canvas.clientHeight / 2) * devicePixelRatio,
      0,
    ])
    const zoom = STATE.zoom * devicePixelRatio
    mat4.scale(projection, projection, [zoom, zoom, zoom])
    mat4.translate(projection, projection, [
      -STATE.ship.location[0],
      -STATE.ship.location[1],
      0,
    ])

    shapes.forEach((shape) => {
      mat4.multiply(view, projection, shape.transform)
      if (shape.type == "circle") {
        vec2.transformMat4(v1, center, view)
        vec2.transformMat4(v2, edge, view)

        ctx.beginPath()
        ctx.arc(
          v1[0],
          v1[1],
          vec2.distance(v1, v2),
          0,
          Math.PI * 2,
          shape.filled
        )
        ctx.strokeStyle = serializeColor(shape.color)
        ctx.stroke()
      }
      if (shape.type == "lines") {
        ctx.beginPath()
        for (let i = 0; i < shape.points.length; i++) {
          vec2.transformMat4(v1, shape.points[i], view)
          ctx.lineTo(v1[0], v1[1])
        }
        ctx.strokeStyle = serializeColor(shape.color)
        ctx.stroke()
      }
    })
  }
}

/**
 * @param {string | number[]} color
 * @returns {number[]}
 */
function parseColor(color) {
  if (color && typeof color[0] === "number") {
    return color
  }
  if (color && color.startsWith("#") && color.length === 4) {
    return [
      parseInt(color[1], 16) / 16,
      parseInt(color[2], 16) / 16,
      parseInt(color[3], 16) / 16,
      1,
    ]
  }
  if (color && color.startsWith("rgba(") && color.endsWith(")")) {
    return color
      .slice(5, color.length - 1)
      .split(",")
      .map((c, i) => parseFloat(c) / (i === 3 ? 1 : 255))
  }
  return [1, 1, 1, 1]
}

/**
 * @param {number[]} color
 * @returns {string}
 */
function serializeColor(color) {
  return `rgba(${color[0] * 255}, ${color[1] * 255}, ${color[2] * 255}, ${
    color[3]
  })`
}

const renderers = [setup2dRender(), setupWebGlRender()]
const debugInfo = document.createElement("div")
debugInfo.id = "debug"
document.body.appendChild(debugInfo)

function render() {
  if (!STATE.rendered) {
    const shapes = worldShapes()
    for (const render of renderers) {
      const start = performance.now()
      render(shapes)
      const time = performance.now() - start
      const pastWeight = 0.95
      render.time = (render.time || time) * pastWeight + time * (1 - pastWeight)
    }
    debugInfo.innerHTML = `2d: ${renderers[0].time.toFixed(
      1
    )}ms<br>webgl: ${renderers[1].time.toFixed(1)}ms`
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
    location: translate(bodyToStartOn.location, [
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

setInterval(tick, 15)
tick(true)
render()
