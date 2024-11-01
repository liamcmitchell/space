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

function debugSlider(label, value) {
  if (!location.hash.includes("debug")) return () => value

  const debugLine = document.createElement("div")
  debugLine.innerText = label
  document.getElementById("debug").appendChild(debugLine)
  const slider = document.createElement("input")
  slider.type = "range"
  slider.min = 0
  slider.max = 2
  slider.step = 0.01
  slider.value = value
  debugLine.appendChild(slider)

  return function () {
    return slider.valueAsNumber
  }
}

/**
 * @typedef {[number, number]} Vector
 */

/**
 * @typedef {object} Thruster
 * @prop {Vector} location
 * @prop {number} angle
 * @prop {number} force
 * @prop {boolean} on
 */

/**
 * @typedef {object} Ship
 * @prop {number} mass
 * @prop {number} inertia
 * @prop {Vector} location
 * @prop {Vector} velocity
 * @prop {number} angle
 * @prop {number} angularVelocity
 * @prop {Thruster[]} thrusters
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
  [-0.5, 0],
  [0, -1],
  [0.5, 0],
]

/** @type {State} */
const STATE = {
  running: document.hasFocus(),
}

function getTotalMass(bodies) {
  return bodies.reduce((memo, body) => memo + body.mass, 0)
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
  return (massA * massB) / distance ** 2
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
  return (((body.mass / Math.PI) * 3) / 2) ** (1 / 3)
}

const _tick = debugTimer("tick: ", function () {
  STATE.rendered = false
  STATE.debugShapes = []

  if (STATE.ship.destroyed && STATE.ship.destroyedFor > 2) {
    reset()
  }

  const ship = STATE.ship

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

  ship.thrusters[0].on = KEYS["ArrowUp"] && !ship.destroyed
  ship.thrusters[1].on = KEYS["ArrowLeft"] && !ship.destroyed
  ship.thrusters[2].on = KEYS["ArrowRight"] && !ship.destroyed

  if (ship.destroyed) {
    // Increment destroyed counter
    ship.destroyedFor += STATE.timeInc
    // Slowly decrease velocity (and player view)
    vec2.scale(ship.velocity, ship.velocity, 0.99)
  }

  if (!ship.destroyed) {
    // Apply gravity to ship
    vec2.scaleAndAdd(
      ship.velocity,
      ship.velocity,
      getForceOn(ship, STATE.bodies),
      STATE.timeInc / ship.mass
    )

    // Apply thrust to ship
    for (const thruster of ship.thrusters) {
      if (!thruster.on) continue

      vec2.scaleAndAdd(
        ship.velocity,
        ship.velocity,
        getAngleVector(ship.angle + thruster.angle, thruster.force),
        1 / ship.mass
      )
      ship.angularVelocity +=
        vec2.dot(
          perpendicular(thruster.location),
          getAngleVector(thruster.angle, -thruster.force)
        ) / ship.inertia
    }
  }

  // Move ship
  vec2.scaleAndAdd(ship.location, ship.location, ship.velocity, STATE.timeInc)
  ship.angle += ship.angularVelocity * STATE.timeInc
  ship.angle %= Math.PI * 2

  ship.colliding = false
  ship.landed = false

  if (!ship.destroyed) {
    for (const body of STATE.bodies) {
      const distance = vec2.distance(body.location, ship.location)
      const radius = getBodyRadius(body)

      if (distance < radius + shipRadius) {
        const transform = createTransform(ship.angle, ship.location, 1)
        const points = shipBody.map((v) => vec2.transformMat4([], v, transform))
        /** @type {[Vector, number][]} */
        const contactPoints = []
        for (let index = 0; index < points.length - 1; index++) {
          const a = points[index]
          const aDistance = vec2.distance(a, body.location)
          if (aDistance < radius) {
            contactPoints.push([a, aDistance])
          }
          const b = points[index + 1]
          const length = vec2.distance(a, b)
          const closest = closestPointOnLine(body.location, a, b, length)
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
          const shipPoint = vec2.subtract([], ship.location, point)
          const shipPointPerp = perpendicular(shipPoint)
          const shipPointAngularVelocity = vec2.scale(
            [],
            shipPointPerp,
            ship.angularVelocity
          )
          const shipRelativeVelocity = vec2.subtract(
            [],
            ship.velocity,
            body.velocity
          )
          const pointRelativeVelocity = vec2.add(
            [],
            shipRelativeVelocity,
            shipPointAngularVelocity
          )
          const bodyPoint = vec2.subtract([], point, body.location)
          const collisionNormal = vec2.normalize([], bodyPoint)
          const colliding = vec2.dot(collisionNormal, pointRelativeVelocity) < 0
          ship.colliding = ship.colliding || colliding
          if (colliding) {
            const elasticity = 0.3
            const impulse =
              vec2.dot(
                vec2.scale([], pointRelativeVelocity, -(1 + elasticity)),
                collisionNormal
              ) /
              (1 / ship.mass +
                vec2.dot(shipPointPerp, collisionNormal) ** 2 / ship.inertia)
            if (impulse > 20) {
              ship.destroyed = true
              ship.destroyedFor = 0
              return
            }
            vec2.scaleAndAdd(
              ship.velocity,
              ship.velocity,
              collisionNormal,
              impulse / ship.mass
            )
            ship.angularVelocity +=
              vec2.dot(
                shipPointPerp,
                vec2.scale([], collisionNormal, impulse)
              ) / ship.inertia
            // Friction
            const collisionPerp = perpendicular(collisionNormal)
            const surfaceVelocity = vec2.dot(
              pointRelativeVelocity,
              collisionPerp
            )
            vec2.scaleAndAdd(
              ship.velocity,
              ship.velocity,
              collisionPerp,
              -surfaceVelocity / 20
            )
          }
          // Shift out of body.
          vec2.scaleAndAdd(
            ship.location,
            ship.location,
            collisionNormal,
            (radius - pointBodyDistance) / 2
          )
        }

        ship.landed =
          ship.colliding &&
          Math.abs(ship.angularVelocity) < 1 &&
          vec2.distance(ship.velocity, body.velocity) < 0.5 &&
          Math.abs(
            getAngleDifference(
              ship.angle,
              vec2.angle(ship.location, body.location)
            )
          ) < 0.5
      }
    }
  }
})

function tick(force) {
  if (STATE.running || force) {
    _tick()
  }
}

/**
 * @param {Vector} target
 * @param {Vector} a
 * @param {Vector} b
 * @param {number} length
 * @returns {Vector}
 */
function closestPointOnLine(target, a, b, length = vec2.distance(a, b)) {
  const dot =
    ((target[0] - a[0]) * (b[0] - a[0]) + (target[1] - a[1]) * (b[1] - a[1])) /
    length ** 2
  return [a[0] + dot * (b[0] - a[0]), a[1] + dot * (b[1] - a[1])]
}

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
        color: [1, 1, 1, 0.3],
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
    const shipTransform = createTransform(ship.angle, ship.location, 1)
    for (const thruster of ship.thrusters) {
      if (!thruster.on) continue
      const transform = createTransform(
        thruster.angle,
        thruster.location,
        thruster.force * 2
      )
      mat4.multiply(transform, shipTransform, transform)
      shapes.push({
        type: "lines",
        points: shipThrust,
        transform,
        color: [1, 1, 0, 1],
        filled: thruster.on,
      })
    }
    // Ship body
    shapes.push({
      type: "lines",
      points: shipBody,
      transform: shipTransform,
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
  const gl = canvas.getContext("webgl2")
  if (gl === null) {
    alert(
      "Unable to initialize WebGL. Your browser or machine may not support it."
    )
  }

  const backgroundProgramInfo = twgl.createProgramInfo(gl, [
    /* glsl */ `#version 300 es
    in vec4 position;
    void main() {
      gl_Position = position;
    }
    `,
    /* glsl */ `#version 300 es
    precision highp float;
    uniform vec4 color;
    uniform float time;
    out vec4 outColor;
    uvec3 pcg3d(uvec3 v) {
      v = v * 1664525u + 1013904223u;
      v.x += v.y*v.z;
      v.y += v.z*v.x;
      v.z += v.x*v.y;
      v ^= v >> 16u;
      v.x += v.y*v.z;
      v.y += v.z*v.x;
      v.z += v.x*v.y;
      return v;
    }
    void main() {
      vec3 random = vec3(pcg3d(uvec3(gl_FragCoord.xyz))) / float(0xffffffffu);
      outColor = color * pow(random.x, 600.0) * (0.9 + (sin((random.y * 6.3) + (time * random.z * 5.0)) * 0.1));
    }
    `,
  ])

  const programInfo = twgl.createProgramInfo(gl, [
    /* glsl */ `#version 300 es
    in vec4 position;
    uniform mat4 projection;
    uniform mat4 view;
    void main() {
      gl_Position = projection * view * position;
    }
    `,
    /* glsl */ `#version 300 es
    precision highp float;
    uniform vec4 color;
    out vec4 outColor;
    void main() {
      outColor = color;
    }
    `,
  ])

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
    background: createBufferInfoFromArrays([1, 1, 1, -1, -1, 1, -1, -1]),
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

    gl.useProgram(backgroundProgramInfo.program)
    twgl.setUniforms(backgroundProgramInfo, {
      time: STATE.time,
      color: [1, 1, 1, 1],
    })
    setBuffersAndAttributes(bufferInfos.background)
    twgl.drawBufferInfo(gl, bufferInfos.background, gl.TRIANGLE_STRIP)

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

function render(force) {
  tick(force === true)
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
    inertia: (2 * 1 * shipRadius ** 2) / 5,
    location: vec2.add([], bodyToStartOn.location, [
      0,
      -getBodyRadius(bodyToStartOn) - shipRadius + 1,
    ]),
    velocity: bodyToStartOn.velocity,
    angle: Math.PI,
    angularVelocity: 0,
    thrusters: [
      { location: [0, 0], angle: 0, force: 0.5, on: false },
      { location: [0, 1.5], angle: Math.PI / 2, force: 0.2, on: false },
      { location: [0, 1.5], angle: -Math.PI / 2, force: 0.2, on: false },
    ],
    landed: false,
  }
}

reset()
render(true)
