/// <reference types="gl-matrix" />
/// <reference types="twgl.js" />

const twgl = /** @type {import("twgl.js")} */ (window.twgl)
const { mat3, vec2, glMatrix } = /** @type {import("gl-matrix")} */ (
  window.glMatrix
)

glMatrix.setMatrixArrayType(Array)

const debug = location.hash === "#debug"

function debugValue(key, value) {
  if (!debug) return

  const id = `debug-${key}`

  if (!document.getElementById(id)) {
    const element = document.createElement("div")
    element.id = id
    document.getElementById("debug").appendChild(element)
  }

  requestAnimationFrame(() => {
    document.getElementById(id).innerHTML = `${key}: ${value}`
  })
}

function debugTimer(label, fn) {
  if (!debug) return fn

  let averageTime = 0

  return function () {
    const start = performance.now()
    const val = fn.apply(this, arguments)
    const time = performance.now() - start
    averageTime = smooth(averageTime || time, time, 0.95)
    debugValue(label, `${averageTime.toFixed(1)}ms`)
    return val
  }
}

function debugSlider(label, value) {
  if (!debug) return () => value

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

const colors = {
  white: [1, 1, 1, 1],
  gray: [0.7, 0.7, 0.7, 1],
  red: [1, 0, 0, 1],
  green: [0, 1, 0, 1],
  blue: [0, 0, 1, 1],
  yellow: [1, 1, 0, 1],
}

const debugLine = [
  [0, 0],
  [1, 0],
]
function debugPoints(color, origin, offset) {
  if (!debug) return

  STATE.debugPoints.push({ color, origin, offset })
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
 * @prop {(boolean) => void} noise
 * @prop {string} key
 * @prop {string} touch
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
 * @prop {number} health
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
 * @prop {number} radius
 * @prop {Vector} location
 * @prop {Vector} velocity
 */

/**
 * @typedef {object} PhysicalBody
 * @prop {'circle' | 'poly'} type
 * @prop {boolean} fixed
 * @prop {number} mass
 * @prop {number} radius
 * @prop {Vector} location
 * @prop {Vector} velocity
 * @prop {Vector} force
 * @prop {number} angle
 * @prop {number} angularVelocity
 * @prop {number} torque
 * @prop {number} inertia
 * @prop {Vector[]} [points]
 */

/**
 * @typedef {object} State
 * @prop {boolean} running
 * @prop {boolean} rendered
 * @prop {number} time
 * @prop {number} timeInc
 * @prop {number} zoom
 * @prop {PhysicalBody[]} bodies
 * @prop {Ship} ship
 */

/** @type {Vector} */
const center = [0, 0]

/** @type {Vector[]} */
const shipBody = [
  [0, 0],
  [-1, -1],
  [2, 0],
  [-1, 1],
]

const shipRadius = Math.max(
  ...shipBody.map((point) => vec2.distance(point, center))
)

/** @type {Vector[]} */
const shipThrust = [
  [1, 0],
  [0, -0.5],
  [0, 0.5],
]

const shipStrength = 20

/** @type {State} */
const STATE = {
  running: document.hasFocus(),
}

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
  STATE.zoom = 20
  STATE.bodies = []
  setSystemBodies(SYSTEM, STATE.time, STATE.bodies)
  const bodyToStartOn = STATE.bodies[5]
  STATE.ship = {
    type: "poly",
    mass: 1,
    radius: shipRadius,
    inertia: (2 * 1 * shipRadius ** 2) / 5,
    location: vec2.add(
      vec2.create(),
      bodyToStartOn.location,
      vec2.fromValues(1, bodyToStartOn.radius + shipRadius - 1)
    ),
    velocity: vec2.clone(bodyToStartOn.velocity),
    angle: Math.PI / 2,
    angularVelocity: 0,
    health: 1,
    points: shipBody,
    thrusters: [
      {
        location: vec2.create(),
        angle: Math.PI,
        force: 0.5,
        on: false,
        noise: createThrusterSource(100, 0.5),
        key: "ArrowUp",
        touch: "center",
      },
      {
        location: vec2.fromValues(1.5, 0),
        angle: Math.PI / 2,
        force: 0.2,
        on: false,
        noise: createThrusterSource(400, 0.1),
        key: "ArrowRight",
        touch: "right",
      },
      {
        location: vec2.fromValues(1.5, 0),
        angle: -Math.PI / 2,
        force: 0.2,
        on: false,
        noise: createThrusterSource(400, 0.1),
        key: "ArrowLeft",
        touch: "left",
      },
    ],
  }
  STATE.ship.thrusters.forEach((thruster) => {
    thruster.transform = createTransform(
      thruster.angle,
      thruster.location,
      thruster.force * 2
    )
  })
  STATE.bodies.push(STATE.ship)
  for (const start of createCirclePositions(40, 10)) {
    STATE.bodies.push({
      type: "poly",
      mass: 0.5,
      radius: 1,
      points: createCirclePositions(3 + (STATE.bodies.length % 4), 1),
      inertia: (2 * 1 * 2 ** 2) / 5,
      location: vec2.add(vec2.create(), bodyToStartOn.location, start),
      velocity: [...bodyToStartOn.velocity],
      angle: STATE.bodies.length,
      angularVelocity: 0,
      health: 1,
    })
  }
  for (const start of createCirclePositions(30, 6)) {
    STATE.bodies.push({
      type: "poly",
      mass: 0.5,
      radius: 1,
      points: createCirclePositions(3 + (STATE.bodies.length % 4), 1),
      inertia: (2 * 1 * 2 ** 2) / 5,
      location: vec2.add(vec2.create(), bodyToStartOn.location, start),
      velocity: [...bodyToStartOn.velocity],
      angle: STATE.bodies.length,
      angularVelocity: 0,
      health: 1,
    })
  }
  for (const start of createCirclePositions(30, 8)) {
    STATE.bodies.push({
      type: "circle",
      mass: 0.5,
      radius: 0.5,
      inertia: (2 * 1 * 0.5 ** 2) / 5,
      location: vec2.add(vec2.create(), bodyToStartOn.location, start),
      velocity: [...bodyToStartOn.velocity],
      angle: 0,
      angularVelocity: 0,
      health: 1,
    })
  }
  STATE.bodies.forEach((body) => {
    body.inertia ??= Infinity
    body.angle ??= 0
    body.angularVelocity ??= 0
    body.force = vec2.create()
    body.torque = 0
    body.points = body.points?.map((point) => vec2.fromValues(...point))
    body.absolutePoints = body.points?.map(() => vec2.create())
    body.absolutePointsTime = -1
  })
}

function createCirclePositions(stops, radius = 1) {
  return Array.from({ length: stops }, (_, i) => [
    radius * Math.cos((i * 2 * Math.PI) / stops),
    radius * Math.sin((i * 2 * Math.PI) / stops),
  ])
}

/**
 *
 * @param {System} system
 * @returns
 */
function getTotalMass(system) {
  let mass = system.mass
  system.children?.forEach((child) => {
    mass += getTotalMass(child)
  })
  return mass
}

function getOrbitVelocity(parentMass, childMass, radius) {
  const force = getGravityForce(parentMass, childMass, radius)
  return Math.sqrt((force / childMass) * radius)
}

/**
 * @param {System} system
 * @param {number} time
 * @param {SystemBody[]} bodies
 * @param {number} [index]
 * @param {SystemBody} [parent]
 */
function setSystemBodies(system, time, bodies, index = 0, parent) {
  let total = 1

  if (!bodies[index]) {
    bodies[index] = {
      type: "circle",
      fixed: true,
      mass: system.mass,
      radius: getBodyRadius(system.mass),
      location: vec2.create(),
      velocity: vec2.create(),
      orbitRadius: system.orbitRadius,
      orbitCenter: parent?.location,
      landed: false,
    }
  }

  const body = bodies[index]

  if (parent) {
    if (!system.orbitVelocity) {
      system.orbitVelocity = getOrbitVelocity(
        parent.mass,
        getTotalMass(system),
        system.orbitRadius
      )
    }
    const angle = (system.orbitVelocity / system.orbitRadius) * time
    body.location[0] = parent.location[0] + Math.cos(angle) * system.orbitRadius
    body.location[1] = parent.location[1] + Math.sin(angle) * system.orbitRadius
    body.velocity[0] =
      parent.velocity[0] + Math.sin(angle) * -system.orbitVelocity
    body.velocity[1] =
      parent.velocity[1] + Math.cos(angle) * system.orbitVelocity
  }

  system.children?.forEach((child) => {
    total += setSystemBodies(child, time, bodies, index + total, body)
  })

  return total
}

/**
 * @param {number} angle
 * @param {number} distance
 * @returns {Vector}
 */
function getAngleVector(angle, distance) {
  return [Math.cos(angle) * distance, Math.sin(angle) * distance]
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
 * @param {number} mass
 * @returns {number}
 */
function getBodyRadius(mass) {
  return (((mass / Math.PI) * 3) / 2) ** (1 / 3)
}

function tick(force) {
  if (STATE.running || force) {
    _tick()
  }
}

const _tick = debugTimer("tick", function () {
  STATE.rendered = false
  STATE.debugPoints = []

  if (STATE.ship.destroyed && STATE.ship.destroyedFor > 2) {
    reset()
  }

  STATE.time += STATE.timeInc

  const { ship, bodies, time, timeInc } = STATE

  setSystemBodies(SYSTEM, time, bodies)

  if (ship.destroyed) {
    ship.destroyedFor += timeInc
  }

  for (let i = 0; i < ship.thrusters.length; i++) {
    const thruster = ship.thrusters[i]

    thruster.on =
      !ship.destroyed &&
      (KEYS[thruster.key] || touchAreas[thruster.touch].touched)

    thruster.noise(thruster.on)

    if (!thruster.on) continue

    vec2.scaleAndAdd(
      ship.force,
      ship.force,
      getAngleVector(ship.angle + thruster.angle, -thruster.force),
      1 / timeInc
    )
    ship.torque +=
      vec2.dot(
        perpendicular([], thruster.location),
        getAngleVector(thruster.angle, -thruster.force)
      ) / timeInc
  }

  const difference = vec2.create()
  const collisionTangent = vec2.create()
  const collisionForce = vec2.create()
  const aPointPerp = vec2.create()
  const bPointPerp = vec2.create()
  const pointRelativeVelocity = vec2.create()

  for (let i = 0; i < bodies.length; i++) {
    const a = bodies[i]
    for (let j = i + 1; j < bodies.length; j++) {
      const b = bodies[j]

      if (a.fixed && b.fixed) continue

      const distance = vec2.distance(a.location, b.location) || 0.0001

      const gravity = getGravityForce(a.mass, b.mass, distance)

      if (gravity > 0.01) {
        vec2.subtract(difference, b.location, a.location)
        vec2.scaleAndAdd(a.force, a.force, difference, gravity / distance)
        vec2.scaleAndAdd(b.force, b.force, difference, -gravity / distance)
      }

      if (distance - a.radius - b.radius > 0) continue

      const colls = collisions(a, b)
      for (let k = 0; k < colls.length; k++) {
        const [contactPoint, collisionNormal, collisionDepth] = colls[k]

        perpendicular(collisionTangent, collisionNormal)
        vec2.zero(collisionForce)

        vec2.subtract(aPointPerp, contactPoint, a.location)
        perpendicular(aPointPerp, aPointPerp)
        vec2.subtract(bPointPerp, contactPoint, b.location)
        perpendicular(bPointPerp, bPointPerp)

        vec2.scale(pointRelativeVelocity, bPointPerp, b.angularVelocity)
        vec2.add(pointRelativeVelocity, pointRelativeVelocity, b.velocity)
        vec2.scaleAndAdd(
          pointRelativeVelocity,
          pointRelativeVelocity,
          aPointPerp,
          -a.angularVelocity
        )
        vec2.subtract(pointRelativeVelocity, pointRelativeVelocity, a.velocity)

        const velocityIntoNormal = vec2.dot(
          collisionNormal,
          pointRelativeVelocity
        )
        const velocityIntoTangent = vec2.dot(
          collisionTangent,
          pointRelativeVelocity
        )

        const separationForce = collisionDepth / (1 / a.mass + 1 / b.mass)
        vec2.scaleAndAdd(
          collisionForce,
          collisionForce,
          collisionNormal,
          separationForce
        )

        // Collision
        if (velocityIntoNormal < 0) {
          // https://gamemath.com/book/dynamics.html#collision_response_with_rotations
          const elasticity = 0.3
          const impulse =
            (velocityIntoNormal * (elasticity + 1)) /
            (1 / a.mass +
              1 / b.mass +
              vec2.dot(aPointPerp, collisionNormal) ** 2 / a.inertia +
              vec2.dot(bPointPerp, collisionNormal) ** 2 / b.inertia)
          vec2.scaleAndAdd(
            collisionForce,
            collisionForce,
            collisionNormal,
            impulse
          )

          if (a === ship || b === ship) {
            const body = a === ship ? b : a
            if (impulse < -0.2) {
              const volume = 1 - 1 / (0.05 * -impulse + 1) ** 2
              collisionNoise(volume)
            }
            ship.health -= Math.max(0, (-impulse - 2) / shipStrength)
            if (ship.health <= 0 && !ship.destroyed) {
              ship.destroyed = true
              ship.destroyedFor = 0
            }
            if (body.orbitCenter) {
              body.landed =
                body.landed ||
                (Math.abs(ship.angularVelocity) < 1 &&
                  vec2.distance(ship.velocity, body.velocity) < 0.5 &&
                  Math.abs(
                    getAngleDifference(
                      ship.angle,
                      lineAngle(body.location, ship.location)
                    )
                  ) < 0.2)
            }
          }
        }

        // Friction
        const maxFriction =
          velocityIntoTangent /
          (1 / a.mass +
            1 / b.mass +
            vec2.dot(aPointPerp, collisionTangent) ** 2 / a.inertia +
            vec2.dot(bPointPerp, collisionTangent) ** 2 / b.inertia)
        const kineticFriction = 0.5
        const frictionForce =
          Math.min(
            Math.abs(maxFriction),
            vec2.length(collisionForce) * kineticFriction
          ) * Math.sign(velocityIntoTangent)
        vec2.scaleAndAdd(
          collisionForce,
          collisionForce,
          collisionTangent,
          frictionForce
        )

        vec2.scaleAndAdd(a.force, a.force, collisionForce, 1 / timeInc)
        a.torque += vec2.dot(aPointPerp, collisionForce) / timeInc
        vec2.scaleAndAdd(b.force, b.force, collisionForce, -1 / timeInc)
        b.torque += -vec2.dot(bPointPerp, collisionForce) / timeInc
      }
    }
  }

  for (let i = 0; i < bodies.length; i++) {
    const body = bodies[i]
    vec2.scaleAndAdd(
      body.velocity,
      body.velocity,
      body.force,
      timeInc / body.mass
    )
    vec2.scaleAndAdd(body.location, body.location, body.velocity, timeInc)
    vec2.zero(body.force)

    body.angularVelocity += (body.torque * timeInc) / body.inertia
    body.angle += body.angularVelocity * timeInc
    body.angle %= Math.PI * 2
    body.torque = 0

    if (debug && isNaN(body.location[0])) {
      debugger
    }
  }

  const newZoom = Math.min(
    20,
    100 *
      Math.sqrt(
        bodies.reduce(
          (m, b) =>
            b === ship
              ? m
              : m +
                (1 /
                  (vec2.distance(ship.location, b.location) -
                    shipRadius -
                    b.radius)) **
                  2,
          0
        )
      )
  )
  STATE.idealZoom = newZoom
  STATE.zoom = smooth(STATE.zoom, newZoom, 0.95)
  STATE.viewRadius =
    Math.sqrt(STATE.width ** 2 + STATE.height ** 2) / 2 / STATE.zoom

  requestAnimationFrame(render)
})

/**
 *
 * @param {PhysicalBody} a
 * @param {PhysicalBody} b
 * @return {[Vector[], Vector[], number]}
 */
function collisions(a, b) {
  const aType = a.type
  const bType = b.type
  if (aType === "circle" && bType === "circle")
    return collisionsCircleCircle(a, b)
  if (aType === "circle" && bType === "poly") return collisionsCirclePoly(a, b)
  if (bType === "circle" && aType === "poly") {
    const collisions = collisionsCirclePoly(b, a)
    for (let i = 0; i < collisions.length; i++) {
      const collision = collisions[i]
      // Correct normal to a->b.
      vec2.scale(collision[1], collision[1], -1)
    }
    return collisions
  }
  if (aType === "poly" && bType === "poly") return collisionsPolyPoly(a, b)
  return []
}

function collisionsCircleCircle(a, b) {
  const distance = vec2.distance(a.location, b.location)
  const depth = distance - a.radius - b.radius

  if (depth >= 0) return []

  const normal = vec2.create()
  vec2.subtract(normal, b.location, a.location)
  vec2.normalize(normal, normal)

  const point = vec2.create()
  vec2.scaleAndAdd(point, a.location, normal, a.radius + depth)

  return [[point, normal, depth]]
}

/**
 *
 * @param {PhysicalBody} a
 * @param {PhysicalBody} b
 * @return {Vector[]}
 */
function collisionsCirclePoly(circle, poly) {
  const points = getPoints(poly)
  const collisions = []

  for (let index = 0; index < points.length; index++) {
    const a = points[index]
    const b = points[(index + 1) % points.length]
    const length = vec2.distance(a, b)
    const closest = closestPointOnLine(circle.location, a, b, length)
    const closestDistance = vec2.distance(circle.location, closest)
    if (vec2.distance(a, circle.location) < circle.radius) {
      const difference = vec2.subtract(vec2.create(), a, circle.location)
      const depth = vec2.length(difference) - circle.radius
      collisions.push([a, vec2.normalize(difference, difference), depth])
    }
    if (
      closestDistance < circle.radius &&
      vec2.distance(a, closest) + vec2.distance(closest, b) < length + 0.01
    ) {
      const difference = vec2.subtract(vec2.create(), closest, circle.location)
      const depth = vec2.length(difference) - circle.radius
      collisions.push([closest, vec2.normalize(difference, difference), depth])
    }
  }

  return collisions
}

function collisionsPolyPoly(a, b) {
  const intersectingPoints = []
  const aPoints = getPoints(a)
  const bPoints = getPoints(b)
  const aInsideB = []
  const bInsideA = []

  for (let i = 0; i < aPoints.length; i++) {
    const a1 = aPoints[i]
    const a2 = aPoints[(i + 1) % aPoints.length]
    for (let j = 0; j < bPoints.length; j++) {
      const b1 = bPoints[j]
      const b2 = bPoints[(j + 1) % bPoints.length]
      const intersection = lineIntersection(
        a1[0],
        a1[1],
        a2[0],
        a2[1],
        b1[0],
        b1[1],
        b2[0],
        b2[1]
      )
      if (intersection) {
        intersectingPoints.push([intersection, i, j, 0])
      }
      if (linePointSideCheck(a1, b1, b2)) {
        aInsideB[i] = !aInsideB[i]
      }
      if (linePointSideCheck(b1, a1, a2)) {
        bInsideA[j] = !bInsideA[j]
      }
    }
  }
  for (let i = 0; i < aInsideB.length; i++) {
    if (aInsideB[i]) {
      intersectingPoints.push([aPoints[i], i, -1])
    }
  }
  for (let j = 0; j < bInsideA.length; j++) {
    if (bInsideA[j]) {
      intersectingPoints.push([bPoints[j], -1, j, 0])
    }
  }

  if (!intersectingPoints.length) return intersectingPoints

  const intersectionAverage = vec2.create()
  for (let i = 0; i < intersectingPoints.length; i++) {
    vec2.add(intersectionAverage, intersectionAverage, intersectingPoints[i][0])
  }
  vec2.scale(
    intersectionAverage,
    intersectionAverage,
    1 / intersectingPoints.length
  )

  for (let i = 0; i < intersectingPoints.length; i++) {
    intersectingPoints[i][3] = lineAngle(
      intersectionAverage,
      intersectingPoints[i][0]
    )
  }
  intersectingPoints.sort((a, b) => a[3] - b[3])

  const normal = vec2.create()
  for (let i = 0; i < intersectingPoints.length; i++) {
    const intersection1 = intersectingPoints[i]
    const intersection2 =
      intersectingPoints[(i + 1) % intersectingPoints.length]
    const p1 = intersection1[0]
    const p2 = intersection2[0]
    const isA =
      intersection1[2] === -1 ||
      intersection2[2] === -1 ||
      (intersection1[1] !== -1 && intersection1[1] === intersection2[1])
    vec2.add(normal, normal, isA ? p1 : p2)
    vec2.subtract(normal, normal, isA ? p2 : p1)
  }
  vec2.normalize(normal, normal)
  perpendicular(normal, normal)

  const intersectionCenter = vec2.create()
  let area = 0
  for (let i = 1; i < intersectingPoints.length - 1; i++) {
    const p1 = intersectingPoints[0][0]
    const p2 = intersectingPoints[i][0]
    const p3 = intersectingPoints[i + 1][0]
    const subArea = triangleArea(p1, p2, p3) || 0.000001
    const center = triangleCenter(p1, p2, p3)
    area += subArea
    vec2.scaleAndAdd(intersectionCenter, intersectionCenter, center, subArea)
  }
  vec2.scale(intersectionCenter, intersectionCenter, 1 / area)

  let smallestX = Infinity
  let largestX = -Infinity
  const normalAngle = vectorAngle(normal)
  for (let i = 0; i < intersectingPoints.length; i++) {
    const point = intersectingPoints[i][0]
    const rotatedX =
      point[0] * Math.cos(-normalAngle) - point[1] * Math.sin(-normalAngle)
    smallestX = Math.min(rotatedX, smallestX)
    largestX = Math.max(rotatedX, largestX)
  }
  const depth = smallestX - largestX

  return [[intersectionCenter, normal, depth]]
}

function triangleArea(p1, p2, p3) {
  return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])
}

function triangleCenter(p1, p2, p3) {
  const center = vec2.create()
  vec2.lerp(center, p2, p3, 0.5)
  vec2.lerp(center, center, p1, 1 / 3)
  return center
}

// https://www.jeffreythompson.org/collision-detection/poly-point.php
function linePointSideCheck(p, a1, a2) {
  return (
    ((a1[1] >= p[1] && a2[1] < p[1]) || (a1[1] < p[1] && a2[1] >= p[1])) &&
    p[0] < ((a2[0] - a1[0]) * (p[1] - a1[1])) / (a2[1] - a1[1]) + a1[0]
  )
}

// https://www.jeffreythompson.org/collision-detection/line-line.php
function lineIntersection(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y) {
  const uA =
    ((b2x - b1x) * (a1y - b1y) - (b2y - b1y) * (a1x - b1x)) /
    ((b2y - b1y) * (a2x - a1x) - (b2x - b1x) * (a2y - a1y))
  const uB =
    ((a2x - a1x) * (a1y - b1y) - (a2y - a1y) * (a1x - b1x)) /
    ((b2y - b1y) * (a2x - a1x) - (b2x - b1x) * (a2y - a1y))

  if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) {
    return vec2.fromValues(a1x + uA * (a2x - a1x), a1y + uA * (a2y - a1y))
  }
}

/**
 *
 * @param {PhysicalBody} body
 * @returns {Vector[]}
 */
function getPoints(body) {
  if (body.absolutePointsTime !== STATE.time) {
    for (let i = 0; i < body.points.length; i++) {
      const point = body.points[i]
      const absolutePoint = body.absolutePoints[i]
      vec2.rotate(absolutePoint, point, center, body.angle)
      vec2.add(absolutePoint, absolutePoint, body.location)
    }
    body.absolutePointsTime = STATE.time
  }

  return body.absolutePoints
}

/**
 *
 * @param {number} prev
 * @param {number} curr
 * @param {number} pastWeight
 * @returns {number}
 */
function smooth(prev, curr, pastWeight = 0.95) {
  return prev * pastWeight + curr * (1 - pastWeight)
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
  return vec2.fromValues(a[0] + dot * (b[0] - a[0]), a[1] + dot * (b[1] - a[1]))
}

/**
 * @param {Vector} out
 * @param {Vector} in
 * @returns {Vector}
 */
function perpendicular(out, a) {
  const x = a[0]
  out[0] = -a[1]
  out[1] = x
  return out
}

/**
 * Returns angle from x axis.
 * @param {Vector} vector
 * @returns {number}
 */
function vectorAngle(vector) {
  return Math.atan2(vector[1], vector[0])
}

/**
 * Returns angle from x axis.
 * @param {Vector} p1
 * @param {Vector} p2
 * @returns {number}
 */
function lineAngle(p1, p2) {
  return Math.atan2(p2[1] - p1[1], p2[0] - p1[0])
}

/**
 * @typedef {object} Shape
 * @prop {'circle' | 'line'} type
 * @prop {mat3} transform
 * @prop {Vector[]} [points]
 * @prop {number[]} color
 * @prop {boolean} filled
 */

function createTransform(angle, translation, scale) {
  const transform = mat3.create()
  mat3.translate(transform, transform, translation)
  mat3.rotate(transform, transform, angle)
  mat3.scale(transform, transform, [scale, scale])
  return transform
}

function setupRender() {
  const canvas = document.createElement("canvas")
  document.body.appendChild(canvas)

  function updateCanvasWidth() {
    STATE.width = canvas.clientWidth
    STATE.height = canvas.clientHeight
    canvas.width = STATE.width * devicePixelRatio
    canvas.height = STATE.height * devicePixelRatio
  }

  updateCanvasWidth()
  window.addEventListener("resize", () => {
    STATE.rendered = false
    updateCanvasWidth()
    requestAnimationFrame(render)
  })

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
    precision highp int;
    uniform vec4 color;
    uniform float time;
    uniform float devicePixelRatio;
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
    float stars(float pixelRatio, float minBrightness, float flicker) {
      uvec3 seed = uvec3(gl_FragCoord.xyz / pixelRatio);
      seed.z = seed.x ^ seed.y;
      vec3 random = vec3(pcg3d(seed)) / float(0xffffffffu);
      float brightness = pow(random.x, 600.0);
      brightness = brightness * step(minBrightness, brightness);
      return brightness * ((1.0 - flicker) + (sin((random.y * 6.3) + (time * random.z * 5.0)) * flicker));
    }
    void main() {
      float closeStars = stars(devicePixelRatio, 0.8, 0.1);
      float farStars = stars(1.0, 0.0, 0.1);
      outColor = color * (closeStars + farStars);
    }
    `,
  ])

  const programInfo = twgl.createProgramInfo(gl, [
    /* glsl */ `#version 300 es
    in vec2 position;
    uniform mat3 view;
    void main() {
      gl_Position = vec4((view * vec3(position, 1)).xy, 0, 1);
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

  const bufferInfoMap = new WeakMap()

  function createBufferInfoFromArrays(data) {
    if (!bufferInfoMap.has(data)) {
      const flattenedData =
        typeof data[0] === "number" ? data : data.flatMap((d) => [...d])
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
    circle: createBufferInfoFromArrays(createCirclePositions(1000)),
  }

  const circleBufferInfos = Array.from({ length: 6 }, (_, i) =>
    createBufferInfoFromArrays(createCirclePositions(2 ** (i + 4)))
  )

  const projection = mat3.create()
  const view = mat3.create()

  let lastBufferInfo
  function setBuffersAndAttributes(bufferInfo) {
    if (lastBufferInfo !== bufferInfo) {
      twgl.setBuffersAndAttributes(gl, programInfo, bufferInfo)
      lastBufferInfo = bufferInfo
    }
  }

  function renderBufferInfo(bufferInfo, view, color, filled) {
    setBuffersAndAttributes(bufferInfo)
    twgl.setUniforms(programInfo, {
      view: view,
      color: color,
    })
    twgl.drawBufferInfo(gl, bufferInfo, filled ? gl.TRIANGLE_FAN : gl.LINE_LOOP)
    STATE.pointsRendered += bufferInfo.numElements
  }

  function renderCircle(location, radius, filled, color) {
    const distanceFromView =
      vec2.distance(STATE.ship.location, location) - radius - STATE.viewRadius
    if (distanceFromView > 0) return

    const detail = Math.min(
      circleBufferInfos.length - 1,
      Math.floor(Math.log2(radius * STATE.zoom) / 2)
    )
    const bufferInfo = circleBufferInfos[detail]
    mat3.copy(view, projection)
    mat3.translate(view, view, location)
    mat3.scale(view, view, [radius, radius])
    renderBufferInfo(bufferInfo, view, color, filled)
  }

  function renderPoly(points, location, size, angle, filled, color) {
    const distanceFromView =
      vec2.distance(STATE.ship.location, location) - size - STATE.viewRadius
    if (distanceFromView > 0) return

    const bufferInfo = createBufferInfoFromArrays(points)

    mat3.copy(view, projection)
    mat3.translate(view, view, location)
    mat3.rotate(view, view, angle)
    mat3.scale(view, view, [size, size])
    renderBufferInfo(bufferInfo, view, color, filled)
  }

  return function render() {
    if (STATE.rendered) return
    STATE.rendered = true
    STATE.pointsRendered = 0

    const { ship, bodies, zoom, width, height } = STATE

    const pixelWidth = width * devicePixelRatio
    const pixelHeight = height * devicePixelRatio

    gl.viewport(0, 0, pixelWidth, pixelHeight)

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
      devicePixelRatio,
    })
    setBuffersAndAttributes(bufferInfos.background)
    twgl.drawBufferInfo(gl, bufferInfos.background, gl.TRIANGLE_STRIP)

    gl.useProgram(programInfo.program)

    mat3.fromScaling(projection, [(2 / width) * zoom, (2 / height) * zoom])
    mat3.translate(projection, projection, [
      -ship.location[0],
      -ship.location[1],
    ])

    bodies.forEach((body) => {
      if (body.type === "circle") {
        if (body.orbitCenter) {
          renderCircle(
            body.orbitCenter,
            body.orbitRadius,
            false,
            [1, 1, 1, 0.3]
          )
        }
        renderCircle(
          body.location,
          body.radius,
          !debug,
          body.landed ? colors.green : colors.gray
        )
      }
      if (body.type === "poly" && body !== ship) {
        renderPoly(
          body.points,
          body.location,
          body.radius,
          body.angle,
          !debug,
          colors.gray
        )
      }
    })

    // Ship
    // Keep ship visible when zoomed out.
    const shipSize = Math.max(1, 10 / Math.max(STATE.idealZoom, STATE.zoom))
    const shipTransform = createTransform(ship.angle, ship.location, shipSize)
    mat3.multiply(shipTransform, projection, shipTransform)
    for (const thruster of ship.thrusters) {
      if (!thruster.on) continue

      mat3.multiply(view, shipTransform, thruster.transform)
      renderBufferInfo(
        createBufferInfoFromArrays(shipThrust),
        view,
        colors.yellow,
        thruster.on
      )
    }
    renderBufferInfo(
      createBufferInfoFromArrays(ship.points),
      shipTransform,
      ship.destroyed
        ? colors.gray
        : [1 - ship.health, ship.health * 0.4, 2 * ship.health, 1],
      true
    )

    STATE.debugPoints?.forEach(({ color, origin, offset }) => {
      if (!offset) {
        renderCircle(origin, 0.1, false, colors[color] || colors.white)
      } else {
        renderPoly(
          debugLine,
          origin,
          vec2.length(offset),
          vectorAngle(offset),
          false,
          colors[color] || colors.white
        )
      }
    })

    const now = performance.now()
    STATE.fps = smooth(STATE.fps || 0, 1000 / (now - (STATE.lastRender || 0)))
    STATE.lastRender = now
    debugValue("FPS", `${STATE.fps.toFixed(1)}`)
    debugValue("points", `${STATE.pointsRendered}`)
  }
}

const render = setupRender()

/** @type {AudioContext} */
let audioContext
/** @type {AudioBuffer} */
let noiseBuffer
const noiseDuration = 1

function initAudio() {
  if (audioContext) return
  audioContext = new AudioContext()
  const { sampleRate } = audioContext

  const noiseBufferSize = sampleRate * noiseDuration
  noiseBuffer = new AudioBuffer({
    length: noiseBufferSize,
    sampleRate: sampleRate,
  })
  const noiseBufferData = noiseBuffer.getChannelData(0)
  for (let i = 0; i < noiseBufferSize; i++) {
    noiseBufferData[i] = Math.random() * 2 - 1
  }
}

/**
 * @param {(amp: GainNode) => () => void} connectSource
 * @returns {(volume: number, attack: number, sustain: number, release: number) => void}
 */
function createNoise(connectSource) {
  /** @type {() => void} */
  let stopSource
  /** @type {GainNode} */
  let amp
  /** @type {ReturnType<typeof setTimeout>} */
  let stopTimeout

  return function play(volume, attack, sustain, release) {
    if (!audioContext) return

    if (volume > 0 || (amp && amp.gain.value > 0)) {
      if (!stopSource) {
        amp = audioContext.createGain()
        amp.gain.value = 0
        amp.connect(audioContext.destination)
        stopSource = connectSource(amp)
      }
      amp.gain.cancelAndHoldAtTime(audioContext.currentTime)
      amp.gain.linearRampToValueAtTime(
        volume,
        audioContext.currentTime + attack
      )
      amp.gain.setValueAtTime(
        volume,
        audioContext.currentTime + attack + sustain
      )
      amp.gain.setTargetAtTime(
        0,
        audioContext.currentTime + attack + sustain,
        release / 3
      )
      clearTimeout(stopTimeout)
      stopTimeout = setTimeout(() => {
        stopSource()
        stopSource = undefined
        amp = undefined
      }, (length + 1) * 1000)
    }
  }
}

function createThrusterSource(frequency, volume) {
  const playNoise = createNoise((amp) => {
    const source = new AudioBufferSourceNode(audioContext, {
      buffer: noiseBuffer,
      loop: true,
      loopEnd: noiseDuration,
    })
    const bandpass = new BiquadFilterNode(audioContext, {
      type: "bandpass",
      frequency,
      Q: 0.5,
      gain: 0.2,
    })
    source.connect(bandpass).connect(amp)
    source.start()
    return () => {
      source.stop
    }
  })

  return function play(on) {
    playNoise(on ? volume : 0, 0.01, 0.1, 0.1)
  }
}

const _collisionNoise = createNoise((amp) => {
  const source = new AudioBufferSourceNode(audioContext, {
    buffer: noiseBuffer,
    loop: true,
    loopEnd: noiseDuration,
  })
  const bandpass = new BiquadFilterNode(audioContext, {
    type: "bandpass",
    frequency: 400,
    Q: 0.5,
  })
  source.connect(bandpass).connect(amp)
  source.start()
  return () => {
    source.stop()
  }
})

function collisionNoise(volume) {
  _collisionNoise(volume, 1 / 60, 1 / 60, 0.2)
}

// Set event listeners
const KEYS = {
  ArrowLeft: false,
  ArrowUp: false,
  ArrowRight: false,
}
document.addEventListener("keydown", (event) => {
  initAudio()
  KEYS[event.key] = true
  switch (event.key) {
    case "r":
      reset()
      tick(true)
      break

    default:
      break
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
  for (const key in KEYS) {
    KEYS[key] = false
  }
})

const touchAreas = {
  left: { top: 0, bottom: 1, left: 0, right: 0.3, touched: false },
  center: { top: 0, bottom: 1, left: 0.32, right: 0.68, touched: false },
  right: { top: 0, bottom: 1, left: 0.7, right: 1, touched: false },
}

/**
 *
 * @param {TouchEvent} event
 */
function handleTouch(event) {
  initAudio()
  const { innerWidth, innerHeight } = window
  for (const area in touchAreas) {
    touchAreas[area].touched = false
  }
  for (const touch of event.touches) {
    const x = touch.clientX / innerWidth
    const y = touch.clientY / innerHeight
    for (const area in touchAreas) {
      const touchArea = touchAreas[area]
      if (
        x > touchArea.left &&
        x < touchArea.right &&
        y > touchArea.top &&
        y < touchArea.bottom
      ) {
        touchArea.touched = true
      }
    }
  }
}
document.addEventListener("touchstart", handleTouch)
document.addEventListener("touchend", handleTouch)
document.addEventListener("touchcancel", handleTouch)
document.addEventListener("touchmove", handleTouch)

reset()
tick(true)
setInterval(tick, 16)
