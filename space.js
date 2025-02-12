/// <reference types="./typings.d.ts" />

const {
  twgl,
  glMatrix: { mat3, vec2, glMatrix },
} = window

glMatrix.setMatrixArrayType(Array)

const timeInc = 1 / 100

const debug = location.hash.startsWith("#debug")

function test(fn) {
  if (debug) setTimeout(fn)
}

function assert(message, received, expected) {
  if (received != expected)
    throw new Error(`${message}: received ${received} but expected ${expected}`)
}

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
    averageTime = smooth(averageTime || time, time)
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
  slider.min = "0"
  slider.max = "2"
  slider.step = "0.01"
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
  orange: [1, 0.5, 0, 1],
}

const debugLine = [
  [0, 0],
  [1, 0],
]

function debugPoints(color, origin, offset) {
  if (!debug) return

  STATE.debugPoints.push({ color, origin, offset })
}

const center = vec2.create()

/** @type {Vec2[]} */
const shipBody = [
  [-1, -1],
  [2, 0],
  [-1, 1],
]

/** @type {Vec2[]} */
const shipThrust = [
  [1, 0],
  [0, -0.5],
  [0, 0.5],
]

const shipStrength = 20

/** @type {State} */
// @ts-ignore
const STATE = {
  running: document.hasFocus(),
}

/** @type {System} */
const system = {
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

/**
 * @param {Partial<DynamicBody>} input
 * @returns {DynamicBody}
 */
function dynamicBody(input) {
  const { area, points, radius, inertia, offset } = input.points
    ? pointsInfo(input.points, input.scale)
    : {}
  const body = {}
  body.fixed = Boolean(input.fixed)
  body.radius = radius ?? input.radius ?? 1
  body.mass = input.mass ?? area ?? Math.PI * body.radius ** 2
  body.health = input.health ?? 1
  body.destroyed = input.destroyed ?? 0
  body.inertia = inertia
    ? inertia * body.mass
    : (body.mass * body.radius ** 2) / 2
  body.location = input.location ?? vec2.create()
  body.velocity = input.velocity ?? vec2.create()
  body.force = vec2.create()
  body.angle = input.angle ?? 0
  body.angularVelocity = input.angularVelocity ?? 0
  body.torque = 0
  /** @type {BodyType} */
  body.type = input.type ?? points ? "poly" : "circle"
  body.points = points
  body.scale = input.scale ?? 1
  body.absolutePoints =
    input.absolutePoints ?? body.points?.map(() => vec2.create())
  body.absolutePointsTime = -1
  for (const key in input) {
    if (!Object.hasOwn(body, key)) {
      body[key] = input[key]
    }
  }
  // If points were normalized, move location to match offset.
  if (offset) {
    vec2.rotate(offset, offset, center, body.angle)
    vec2.scaleAndAdd(body.location, body.location, offset, body.scale)
  }
  return body
}

/**
 * @param {Vec2[]} points
 * @param {number} scale
 */
function pointsInfo(points, scale = 1) {
  const center = vec2.create()
  let area = 0
  /** @type {{subCenter: Vec2, subArea: number, subInertia: number}[]} */
  const components = []

  for (let i = 1; i < points.length - 1; i++) {
    const p1 = points[0]
    const p2 = points[i]
    const p3 = points[i + 1]
    const subArea = triangleArea(p1, p2, p3) || 0.000001
    const subCenter = vec2.create()
    const subInertia =
      ((vec2.squaredDistance(p1, p2) +
        vec2.squaredDistance(p2, p3) +
        vec2.squaredDistance(p3, p1)) *
        subArea) /
      36
    triangleCenter(subCenter, p1, p2, p3)
    components.push({ subCenter, subArea, subInertia })
    area += subArea
    vec2.scaleAndAdd(center, center, subCenter, subArea)
  }
  vec2.scale(center, center, 1 / area)

  const radius = Math.max(
    ...points.map((point) => vec2.distance(point, center))
  )

  const inertia = components.reduce(
    (acc, { subCenter, subArea, subInertia }) =>
      acc + subInertia + subArea * vec2.sqrDist(center, subCenter),
    0
  )

  // Points should always be centered on 0.
  if (vec2.length(center) > 0.01) {
    points = points.map((point) => vec2.subtract(vec2.create(), point, center))
  }

  return {
    area: area * scale ** 2,
    radius: radius * scale,
    inertia: inertia * scale ** 4,
    offset: center,
    points,
  }
}

test(() => {
  const abs = pointsInfo(createCirclePositions(4, 2))
  const scaled = pointsInfo(createCirclePositions(4, 1), 2)
  assert("scaled area is the same", scaled.area, abs.area)
  assert("scaled inertia is the same", scaled.inertia, abs.inertia)
})

function reset() {
  STATE.rendered = false
  STATE.time = 0
  STATE.zoom = 20
  STATE.bodies = []
  setSystemBodies(system, 0, STATE.bodies)
  const bodyToStartOn = STATE.bodies[5]
  STATE.ship = Object.assign(
    dynamicBody({
      location: vec2.add(vec2.create(), bodyToStartOn.location, [
        0,
        bodyToStartOn.radius + 1,
      ]),
      velocity: vec2.clone(bodyToStartOn.velocity),
      angle: Math.PI / 2,
      points: shipBody,
    }),
    {
      thrusters: [
        {
          location: vec2.fromValues(-1, 0),
          angle: Math.PI,
          force: 100,
          key: "ArrowUp",
          touch: "center",
        },
        {
          location: vec2.fromValues(1.5, 0),
          angle: Math.PI / 2,
          force: 20,
          key: "ArrowRight",
          touch: "right",
        },
        {
          location: vec2.fromValues(1.5, 0),
          angle: -Math.PI / 2,
          force: 20,
          key: "ArrowLeft",
          touch: "left",
        },
      ].map((thruster) => ({
        ...thruster,
        on: false,
        noise: createThrusterSource(thruster.force),
        transform: createTransform(
          thruster.angle,
          thruster.location,
          Math.sqrt(thruster.force) / 10
        ),
      })),
    }
  )
  STATE.bodies.push(STATE.ship)

  const beltRadius = 40
  const orbitVel = orbitVelocity(bodyToStartOn.mass, 40, beltRadius)

  for (const start of createCirclePositions(40, beltRadius)) {
    const angle = vectorAngle(start)
    STATE.bodies.push(
      dynamicBody({
        points: createCirclePositions(3 + (STATE.bodies.length % 5), 1),
        scale: 3 / ((STATE.bodies.length % 3) + 3),
        location: vec2.add(vec2.create(), bodyToStartOn.location, start),
        velocity: vec2.add(vec2.create(), bodyToStartOn.velocity, [
          Math.sin(angle) * -orbitVel,
          Math.cos(angle) * orbitVel,
        ]),
        angle: STATE.bodies.length,
      })
    )
  }
}

const circles = new Map()
function createCirclePositions(stops, radius = 1) {
  const key = `${stops},${radius}`
  if (!circles.has(key)) {
    circles.set(
      key,
      Array.from({ length: stops }, (_, i) =>
        vec2.fromValues(
          radius * Math.cos((i * 2 * Math.PI) / stops),
          radius * Math.sin((i * 2 * Math.PI) / stops)
        )
      )
    )
  }
  return circles.get(key)
}

/**
 *
 * @param {System} system
 * @returns
 */
function totalMass(system) {
  let mass = system.mass
  system.children?.forEach((child) => {
    mass += totalMass(child)
  })
  return mass
}

function orbitVelocity(parentMass, childMass, radius) {
  const force = gravityForce(parentMass, childMass, radius)
  return Math.sqrt((force / childMass) * radius)
}

/**
 * @param {System} system
 * @param {number} time
 * @param {DynamicBody[]} bodies
 * @param {number} [index]
 * @param {DynamicBody} [parent]
 */
function setSystemBodies(system, time, bodies, index = 0, parent) {
  let total = 1

  if (!bodies[index]) {
    bodies[index] = dynamicBody({
      fixed: true,
      mass: system.mass,
      radius: bodyRadius(system.mass),
      orbitRadius: system.orbitRadius,
      orbitCenter: parent?.location,
      landed: false,
    })

    if (!system.orbitVelocity && parent) {
      system.orbitVelocity = orbitVelocity(
        parent.mass,
        totalMass(system),
        system.orbitRadius
      )
    }
  }

  const body = bodies[index]

  if (parent) {
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
 * @returns {Vec2}
 */
function angleVector(angle, distance) {
  return [Math.cos(angle) * distance, Math.sin(angle) * distance]
}

/**
 * @param {number} a
 * @param {number} b
 * @returns {number}
 */
function angleDifference(a, b) {
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
function gravityForce(massA, massB, distance) {
  return (massA * massB) / distance ** 2
}

/**
 * @param {number} mass
 * @returns {number}
 */
function bodyRadius(mass) {
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

  if (STATE.ship.destroyed && STATE.time - STATE.ship.destroyed > 120) {
    reset()
  }

  STATE.time += 1

  const { ship, bodies, time } = STATE

  for (let i = 0; i < ship.thrusters.length; i++) {
    const thruster = ship.thrusters[i]

    thruster.on =
      !ship.destroyed &&
      (KEYS[thruster.key] || touchAreas[thruster.touch].touched)

    thruster.noise(thruster.on)

    if (!thruster.on) continue

    vec2.add(
      ship.force,
      ship.force,
      angleVector(ship.angle + thruster.angle, -thruster.force)
    )
    ship.torque += vec2.dot(
      perpendicular(vec2.create(), thruster.location),
      angleVector(thruster.angle, -thruster.force)
    )
  }

  const difference = vec2.create()
  const collisionTangent = vec2.create()
  const collisionForce = vec2.create()
  const aPointPerp = vec2.create()
  const bPointPerp = vec2.create()
  const pointRelativeVelocity = vec2.create()
  const bodiesLength = bodies.length

  for (let i = 0; i < bodiesLength; i++) {
    const a = bodies[i]

    if (a.destroyed) continue

    for (let j = i + 1; j < bodiesLength; j++) {
      const b = bodies[j]

      if ((a.fixed && b.fixed) || b.destroyed) continue

      const distance = vec2.distance(a.location, b.location) || 0.0001

      const gravity = gravityForce(a.mass, b.mass, distance)

      if (gravity > 0.01) {
        vec2.subtract(difference, b.location, a.location)
        vec2.scaleAndAdd(a.force, a.force, difference, gravity / distance)
        vec2.scaleAndAdd(b.force, b.force, difference, -gravity / distance)
      }

      if (distance - a.radius - b.radius > 0) continue

      const colls = collisions(a, b, distance)
      for (let k = 0; k < colls.length; k++) {
        const {
          point: contactPoint,
          normal: collisionNormal,
          depth: collisionDepth,
        } = colls[k]

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

        // Calculate collision response and separation force together.
        // https://gamemath.com/book/dynamics.html#collision_response_with_rotations
        const elasticity = 0.3
        const collisionResponseVelocity = Math.min(
          0,
          velocityIntoNormal * (elasticity + 1)
        )
        const separationTime = timeInc * 10
        const separationVelocity = collisionDepth / separationTime
        const impulse =
          (collisionResponseVelocity + separationVelocity) /
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

        if (velocityIntoNormal < 0) {
          a.health -= Math.max(0, (-impulse - 2) / 20 / a.mass)
          b.health -= Math.max(0, (-impulse - 2) / 20 / b.mass)

          if (a === ship || b === ship) {
            const relativeImpulse = impulse / ship.mass
            const body = a === ship ? b : a

            if (relativeImpulse < -0.3) {
              const volume = 1 - 1 / (0.05 * -relativeImpulse + 1) ** 2
              collisionNoise(volume)
            }

            if (body.orbitCenter) {
              body.landed =
                body.landed ||
                (Math.abs(ship.angularVelocity) < 1 &&
                  vec2.distance(ship.velocity, body.velocity) < 0.5 &&
                  Math.abs(
                    angleDifference(
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
          Math.min(Math.abs(maxFriction), -impulse * kineticFriction) *
          Math.sign(velocityIntoTangent)
        vec2.scaleAndAdd(
          collisionForce,
          collisionForce,
          collisionTangent,
          frictionForce
        )

        // Collision impulses applied immediately to avoid explosions.
        vec2.scaleAndAdd(a.velocity, a.velocity, collisionForce, 1 / a.mass)
        a.angularVelocity += vec2.dot(aPointPerp, collisionForce) / a.inertia
        vec2.scaleAndAdd(b.velocity, b.velocity, collisionForce, -1 / b.mass)
        b.angularVelocity += -vec2.dot(bPointPerp, collisionForce) / b.inertia
      }
    }
  }

  for (let i = 0; i < bodies.length; i++) {
    const body = bodies[i]
    if (body.health < 0 && !body.destroyed) {
      body.destroyed = time
      if (body.mass > 0.3) {
        const parts = explode(body)
        bodies[i] = parts[0]
        bodies.push(...parts.slice(1))
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

  setSystemBodies(system, time * timeInc, bodies)

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
                    ship.radius -
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
 * @param {DynamicBody} a
 * @param {DynamicBody} b
 * @param {number} distance
 * @return {Collision[]}
 */
function collisions(a, b, distance) {
  const aType = a.type
  const bType = b.type
  if (aType === "circle" && bType === "circle") {
    return collisionsCircleCircle(a, b, distance)
  }
  if (aType === "circle" && bType === "poly") {
    return collisionsCirclePoly(a, b)
  }
  if (bType === "circle" && aType === "poly") {
    const collisions = collisionsCirclePoly(b, a)
    for (let i = 0; i < collisions.length; i++) {
      const normal = collisions[i].normal
      // Correct normal to a->b.
      vec2.scale(normal, normal, -1)
    }
    return collisions
  }
  if (aType === "poly" && bType === "poly") {
    return collisionsPolyPoly(a, b)
  }
  return []
}

/**
 * @param {DynamicBody} a
 * @param {DynamicBody} b
 * @param {number} distance
 * @return {Collision[]}
 */
function collisionsCircleCircle(a, b, distance) {
  const depth = distance - a.radius - b.radius

  const normal = vec2.create()
  vec2.subtract(normal, b.location, a.location)
  vec2.normalize(normal, normal)

  const point = vec2.create()
  vec2.scaleAndAdd(point, a.location, normal, a.radius + depth)

  return [{ point, normal, depth }]
}

/**
 * @param {DynamicBody} circle
 * @param {DynamicBody} poly
 * @return {Collision[]}
 */
function collisionsCirclePoly(circle, poly) {
  const { radius, location } = circle
  const cx = location[0]
  const cy = location[1]
  const points = absolutePoints(poly)
  /** @type {Vec2[]} */
  const intersections = []

  for (let index = 0; index < points.length; index++) {
    const a = points[index]
    const b = points[(index + 1) % points.length]

    const aInside = vec2.distance(a, location) < radius
    if (aInside) {
      intersections.push(a)
    }

    // https://mathworld.wolfram.com/Circle-LineIntersection.html
    const dx = b[0] - a[0]
    const dy = b[1] - a[1]
    const dr = Math.sqrt(dx ** 2 + dy ** 2) // length
    const dr2 = dr ** 2
    const d = (a[0] - cx) * (b[1] - cy) - (a[1] - cy) * (b[0] - cx)
    const discriminant = radius ** 2 * dr2 - d ** 2
    if (discriminant > 0) {
      const ddy = d * dy
      const ddx = d * dx
      const dsqrt = Math.sqrt(discriminant)
      const xoffset = Math.sign(dy) * dx * dsqrt
      const x1 = cx + (ddy + xoffset) / dr2
      const x2 = cx + (ddy - xoffset) / dr2
      const yoffset = Math.abs(dy) * dsqrt
      const y1 = cy + (-ddx + yoffset) / dr2
      const y2 = cy + (-ddx - yoffset) / dr2
      const i1 = vec2.fromValues(x1, y1)
      const i2 = vec2.fromValues(x2, y2)
      const i1Distance = vec2.distance(a, i1)
      const i2Distance = vec2.distance(a, i2)
      const i1Intersecting = i1Distance + vec2.distance(i1, b) < dr + 0.01
      const i2Intersecting = i2Distance + vec2.distance(i2, b) < dr + 0.01
      const i1Closer = i1Distance < i2Distance

      // When entering the circle, we need to add at least one point representing the circle curve from the last exit.
      // We add a placeholder here and calculate the real position when we have all points.
      if (!aInside && i1Intersecting && i2Intersecting) {
        // entering and exiting
        intersections.push(vec2.create())
        intersections.push(i1Closer ? i1 : i2)
        intersections.push(i1Closer ? i2 : i1)
      } else if (i1Intersecting || i2Intersecting) {
        const next = i1Intersecting ? i1 : i2
        if (!aInside) {
          // entering
          intersections.push(vec2.create())
          intersections.push(next)
        } else {
          // exiting
          intersections.push(next)
        }
      }
    }
  }

  if (intersections.length < 3) return []

  // Replace placeholders with actual curve points.
  for (let i = 0; i < intersections.length; i++) {
    const point = intersections[i]
    if (point[0] === 0 && point[1] === 0) {
      const p1 = intersections.at(i - 1)
      const p2 = intersections.at(i + 1)
      const angle1 = vectorAngle(vec2.subtract(point, location, p1))
      const angle2 = vectorAngle(vec2.subtract(point, location, p2))
      const curveAngle = angleDifference(angle1, angle2)
      vec2.rotate(point, p1, location, curveAngle / 2)
    }
  }

  // Calculate precise center using triangles.
  const center = vec2.create()
  const subCenter = vec2.create()
  let area = 0
  for (let i = 1; i < intersections.length - 1; i++) {
    const p1 = intersections[0]
    const p2 = intersections[i]
    const p3 = intersections[i + 1]
    const subArea = triangleArea(p1, p2, p3) || 0.000001
    triangleCenter(subCenter, p1, p2, p3)
    area += subArea
    vec2.scaleAndAdd(center, center, subCenter, subArea)
  }
  vec2.scale(center, center, 1 / area)

  const normalAngle = lineAngle(location, center)
  const normal = angleVector(normalAngle, 1)

  let smallestX = Infinity
  let largestX = -Infinity
  for (let i = 0; i < intersections.length; i++) {
    const point = intersections[i]
    const rotatedX =
      point[0] * Math.cos(-normalAngle) - point[1] * Math.sin(-normalAngle)
    smallestX = Math.min(rotatedX, smallestX)
    largestX = Math.max(rotatedX, largestX)
  }
  const depth = smallestX - largestX

  return [{ point: center, normal, depth }]
}

/**
 * @param {DynamicBody} a
 * @param {DynamicBody} b
 * @return {Collision[]}
 */
function collisionsPolyPoly(a, b) {
  /** @type {{point: Vec2, ai: number, bi: number, angle: number}[]} */
  const intersections = []
  const aPoints = absolutePoints(a)
  const bPoints = absolutePoints(b)
  const aLength = aPoints.length
  const bLength = bPoints.length
  const aInsideB = []
  const bInsideA = []

  for (let ai = 0; ai < aLength; ai++) {
    const { 0: a1x, 1: a1y } = aPoints[ai]
    const { 0: a2x, 1: a2y } = aPoints[(ai + 1) % aLength]
    for (let bi = 0; bi < bLength; bi++) {
      const { 0: b1x, 1: b1y } = bPoints[bi]
      const { 0: b2x, 1: b2y } = bPoints[(bi + 1) % bLength]
      const point = lineIntersection(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y)
      if (point) {
        intersections.push({ point, ai, bi, angle: 0 })
      }
      if (linePointSideCheck(a1x, a1y, b1x, b1y, b2x, b2y)) {
        aInsideB[ai] = !aInsideB[ai]
      }
      if (linePointSideCheck(b1x, b1y, a1x, a1y, a2x, a2y)) {
        bInsideA[bi] = !bInsideA[bi]
      }
    }
  }
  for (let ai = 0; ai < aLength; ai++) {
    if (aInsideB[ai]) {
      intersections.push({ point: aPoints[ai], ai, bi: -1, angle: 0 })
    }
  }
  for (let bi = 0; bi < bLength; bi++) {
    if (bInsideA[bi]) {
      intersections.push({ point: bPoints[bi], ai: -1, bi, angle: 0 })
    }
  }

  const iLength = intersections.length
  if (!iLength) return []

  // Calculate rough center by averaging intersecting points.
  const center = vec2.create()
  for (let i = 0; i < iLength; i++) {
    vec2.add(center, center, intersections[i].point)
  }
  vec2.scale(center, center, 1 / iLength)

  // Calculate angle to rough center.
  for (let i = 0; i < iLength; i++) {
    const intersection = intersections[i]
    intersection.angle = lineAngle(center, intersection.point)
  }
  // Sort by angle (counter-clockwise).
  intersections.sort((a, b) => a.angle - b.angle)

  // Calculate normal by adding all A lines and subtracting all B.
  const normal = vec2.create()
  for (let i = 0; i < iLength; i++) {
    const { point: p1, ai: p1a, bi: p1b } = intersections[i]
    const { point: p2, ai: p2a, bi: p2b } = intersections[(i + 1) % iLength]
    const isA = p1b === -1 || p2b === -1 || (p1a !== -1 && p1a === p2a)
    vec2.add(normal, normal, isA ? p1 : p2)
    vec2.subtract(normal, normal, isA ? p2 : p1)
  }

  if (normal[0] === 0 && normal[1] === 0) return []

  vec2.normalize(normal, normal)
  perpendicular(normal, normal)

  // Calculate precise center using triangles.
  vec2.zero(center)
  const subCenter = vec2.create()
  let area = 0
  for (let i = 1; i < iLength - 1; i++) {
    const p1 = intersections[0].point
    const p2 = intersections[i].point
    const p3 = intersections[i + 1].point
    const subArea = triangleArea(p1, p2, p3)
    triangleCenter(subCenter, p1, p2, p3)
    area += subArea
    vec2.scaleAndAdd(center, center, subCenter, subArea)
  }
  vec2.scale(center, center, 1 / area)

  if (area <= 0) return []

  // Calculate depth by rotating points to normal and finding smallest/largest x.
  let smallestX = Infinity
  let largestX = -Infinity
  const normalAngle = vectorAngle(normal)
  for (let i = 0; i < iLength; i++) {
    const point = intersections[i].point
    const rotatedX =
      point[0] * Math.cos(-normalAngle) - point[1] * Math.sin(-normalAngle)
    smallestX = Math.min(rotatedX, smallestX)
    largestX = Math.max(rotatedX, largestX)
  }
  const depth = smallestX - largestX

  return [{ point: center, normal, depth }]
}

// https://en.wikipedia.org/wiki/Area_of_a_triangle#Using_coordinates
function triangleArea(p1, p2, p3) {
  return (
    0.5 *
    ((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0]))
  )
}

test(() => {
  assert("half of square", triangleArea([0, 0], [1, 0], [0, 1]), 0.5)
})

function triangleCenter(out, p1, p2, p3) {
  vec2.lerp(out, p2, p3, 0.5)
  vec2.lerp(out, out, p1, 1 / 3)
  return out
}

// https://www.jeffreythompson.org/collision-detection/poly-point.php
function linePointSideCheck(px, py, a1x, a1y, a2x, a2y) {
  return (
    ((a1y >= py && a2y < py) || (a1y < py && a2y >= py)) &&
    px < ((a2x - a1x) * (py - a1y)) / (a2y - a1y) + a1x
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
 * @param {DynamicBody} body
 * @returns {Vec2[]}
 */
function absolutePoints(body) {
  if (body.absolutePointsTime !== STATE.time) {
    for (let i = 0; i < body.points.length; i++) {
      const point = body.points[i]
      const absolutePoint = body.absolutePoints[i]
      vec2.rotate(absolutePoint, point, center, body.angle)
      vec2.scale(absolutePoint, absolutePoint, body.scale)
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
function smooth(prev, curr, pastWeight = 0.99) {
  return prev * pastWeight + curr * (1 - pastWeight)
}

/**
 * @param {Vec2} target
 * @param {Vec2} a
 * @param {Vec2} b
 * @param {number} length
 * @returns {Vec2}
 */
function closestPointOnLine(target, a, b, length = vec2.distance(a, b)) {
  const dot =
    ((target[0] - a[0]) * (b[0] - a[0]) + (target[1] - a[1]) * (b[1] - a[1])) /
    length ** 2
  return [a[0] + dot * (b[0] - a[0]), a[1] + dot * (b[1] - a[1])]
}

/**
 * @param {Vec2} out
 * @param {Vec2} a
 * @returns {Vec2}
 */
function perpendicular(out, a) {
  const x = a[0]
  out[0] = -a[1]
  out[1] = x
  return out
}

/**
 * Returns angle from x axis.
 * @param {Vec2} vector
 * @returns {number}
 */
function vectorAngle(vector) {
  return Math.atan2(vector[1], vector[0])
}

/**
 * Returns angle from x axis.
 * @param {Vec2} p1
 * @param {Vec2} p2
 * @returns {number}
 */
function lineAngle(p1, p2) {
  return Math.atan2(p2[1] - p1[1], p2[0] - p1[0])
}

/**
 * @param {DynamicBody} body
 */
function explode(body) {
  if (body.type !== "poly") return [body]

  const { points } = body
  const parts = []

  if (points.length === 3) {
    // Split into 4, re-using original points.
    parts.push(
      dynamicBody({
        points,
        scale: body.scale / 2,
        location: vec2.clone(body.location),
        angle: body.angle + Math.PI,
        angularVelocity: body.angularVelocity,
      })
    )
    const absPoints = absolutePoints(body)
    for (let i = 0; i < absPoints.length; i++) {
      parts.push(
        dynamicBody({
          points,
          scale: body.scale / 2,
          location: vec2.lerp(vec2.create(), body.location, absPoints[i], 0.5),
          angle: body.angle,
          angularVelocity: body.angularVelocity,
        })
      )
    }
  }

  if (parts.length === 0) {
    for (let i = 0; i < points.length; i++) {
      // Make triangles from each edge to center.
      parts.push(
        dynamicBody({
          // For symmetrical points, a single points could be re-used and rotated.
          points: [points.at(i - 1), points[i], center],
          scale: body.scale,
          location: vec2.clone(body.location),
          angle: body.angle,
          angularVelocity: body.angularVelocity,
        })
      )
    }
  }

  // Adjust velocity from angularVelocity & offset from center.
  parts.forEach((bodyPart) => {
    vec2.subtract(bodyPart.velocity, bodyPart.location, body.location)
    perpendicular(bodyPart.velocity, bodyPart.velocity)
    vec2.scale(bodyPart.velocity, bodyPart.velocity, body.angularVelocity)
    vec2.add(bodyPart.velocity, bodyPart.velocity, body.velocity)
  })

  return parts
}

/**
 * @typedef {object} Shape
 * @prop {'circle' | 'line'} type
 * @prop {mat3} transform
 * @prop {Vec2[]} [points]
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
      return brightness * ((1.0 - flicker) + (sin((random.y * 6.3) + (time * random.z / 20.0)) * flicker));
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
        renderPoly(body.points, body.location, body.scale, body.angle, !debug, [
          0.7,
          0.7 - (1 - body.health) / 2,
          body.health * 0.7,
          !body.destroyed ? 1 : 1 - (STATE.time - body.destroyed) / 120,
        ])
      }
      if (body === ship) {
        // Keep ship visible when zoomed out.
        const shipSize = Math.max(1, 10 / Math.max(STATE.idealZoom, STATE.zoom))
        const shipTransform = createTransform(
          ship.angle,
          ship.location,
          shipSize
        )
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
      }
    })

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

/**
 * @param {number} force
 */
function createThrusterSource(force) {
  const frequency = 1000 / Math.sqrt(force) + 200
  const volume = 1 - 1 / (0.1 * force + 1)
  const playNoise = createNoise((amp) => {
    const source = new AudioBufferSourceNode(audioContext, {
      buffer: noiseBuffer,
      loop: true,
      loopEnd: noiseDuration,
    })
    const bandpass = new BiquadFilterNode(audioContext, {
      type: "lowpass",
      frequency,
      Q: 1,
    })
    source.connect(bandpass).connect(amp)
    source.start()
    return () => {
      source.stop()
    }
  })

  return function play(on) {
    playNoise(on ? volume : 0, 0.05, 0.1, 0.1)
  }
}

const _collisionNoise = createNoise((amp) => {
  const source = new AudioBufferSourceNode(audioContext, {
    buffer: noiseBuffer,
    loop: true,
    loopEnd: noiseDuration,
  })
  const bandpass = new BiquadFilterNode(audioContext, {
    type: "lowpass",
    frequency: 1200,
    Q: 0.8,
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

    case "p":
      STATE.running = !STATE.running

    case " ":
      if (debug && !STATE.running) {
        tick(true)
      }

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
