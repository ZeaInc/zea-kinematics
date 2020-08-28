import {
  Quat,
  NumberParameter,
  Operator,
  OperatorInput,
  OperatorOutput,
  Registry,
  Xfo,
  Vec3,
  Material,
  GeomItem,
  TreeItem,
  Lines,
  Color
} from '@zeainc/zea-engine'

const X_AXIS = new Vec3(1, 0, 0)
const Y_AXIS = new Vec3(0, 1, 0)
const Z_AXIS = new Vec3(0, 0, 1)
const identityXfo = new Xfo()
const identityQuat = new Quat()

const generateDebugLines = (debugTree, color) => {
  const line = new Lines()
  const linepositions = line.getVertexAttribute('positions')

  const mat = new Material('debug', 'LinesShader')
  mat.getParameter('BaseColor').setValue(new Color(color))
  mat.getParameter('Overlay').setValue(1)

  const debugGeomItem = new GeomItem('Pointer', line, mat)
  debugTree.addChild(debugGeomItem)

  let numDebugSegments = 0
  let numDebugPoints = 0

  return {
    addDebugSegment: (p0, p1) => {
      const pid0 = numDebugPoints
      const pid1 = numDebugPoints + 1
      numDebugSegments++
      numDebugPoints += 2
      if (line.getNumVertices() < numDebugPoints) line.setNumVertices(numDebugPoints)
      if (line.getNumSegments() < numDebugSegments) line.setNumSegments(numDebugSegments)
      line.setSegmentVertexIndices(numDebugSegments - 1, pid0, pid1)
      linepositions.getValueRef(pid0).setFromOther(p0)
      linepositions.getValueRef(pid1).setFromOther(p1)
    },
    doneFrame: () => {
      line.emit('geomDataTopologyChanged')
      numDebugSegments = 0
      numDebugPoints = 0
    }
  }
}

class IKJoint {
  constructor(index, axisId = 0, solverDebugTree) {
    this.index = index
    this.axisId = axisId
    this.limits = [-Math.PI, Math.PI]
    this.align = new Quat()

    this.debugTree = new TreeItem('IKJoint' + index)
    solverDebugTree.addChild(this.debugTree)
    this.debugLines = {}
  }

  addDebugSegment(color, p0, p1) {
    if (!this.debugLines[color]) {
      this.debugLines[color] = generateDebugLines(this.debugTree, color)
    }
    this.debugLines[color].addDebugSegment(p0, p1)
  }

  init(parentXfo) {
    this.xfo = this.output.getValue().clone() // until we have an IO output
    this.localXfo = parentXfo.inverse().multiply(this.xfo)

    switch (this.axisId) {
      case 0:
        this.axis = X_AXIS
        break
      case 1:
        this.axis = Y_AXIS
        break
      case 2:
        this.axis = Z_AXIS
        break
    }
  }

  evalCCD(baseXfo, targetXfo, index, joints) {
    if (index == joints.length - 1) {
      this.xfo.ori = targetXfo.ori.clone()
    } else {
      {
        const targetVec = targetXfo.tr.subtract(this.xfo.tr)
        const jointToTip = joints[joints.length - 1].xfo.tr.subtract(this.xfo.tr)
        this.align.setFrom2Vectors(jointToTip.normalize(), targetVec.normalize())
        this.xfo.ori = this.align.multiply(this.xfo.ori)
        // this.addDebugSegment('#FF0000', this.xfo.tr, this.xfo.tr.add(jointToTip))
        // this.addDebugSegment('#FFFF00', this.xfo.tr, this.xfo.tr.add(targetVec))
      }
    }

    ///////////////////////
    // Apply joint constraint.
    if (index > 0) {
      this.align.setFrom2Vectors(this.xfo.ori.rotateVec3(this.axis), joints[index - 1].xfo.ori.rotateVec3(this.axis))
      const parentJoint = joints[index - 1]
      const parentAlign = this.align.conjugate()
      // parentJoint.xfo.ori = parentAlign.multiply(parentJoint.xfo.ori)
      if (index == joints.length - 1) {
        parentJoint.xfo.ori = parentAlign.multiply(parentJoint.xfo.ori)
      } else {
        parentJoint.xfo.ori = parentAlign.lerp(identityQuat, 0.5).multiply(parentJoint.xfo.ori)
        this.xfo.ori = this.align.lerp(identityQuat, 0.5).multiply(this.xfo.ori)
      }
    } else {
      this.align.setFrom2Vectors(this.xfo.ori.rotateVec3(this.axis), baseXfo.ori.rotateVec3(this.axis))
      this.xfo.ori = this.align.multiply(this.xfo.ori)
    }

    ///////////////////////
    // Apply angle Limits.

    // const currAngle = Math.acos(this.xfo.ori.dot(parentXfo.ori))
    // if (currAngle < this.limits[0] || currAngle > this.limits[1]) {
    // const globalAxis = this.xfo.ori.rotateVec3(this.axis)
    //   const deltaAngle =
    //     currAngle < this.limits[0] ? this.limits[0] - currAngle : currAngle - this.limits[1]
    //   this.align.setFromAxisAndAngle(globalAxis, deltaAngle)
    //   this.xfo.ori = this.align.multiply(this.xfo.ori)
    // }

    this.xfo.ori.normalizeInPlace()

    if (index > 0) {
      this.localXfo.ori = joints[index - 1].xfo.ori.inverse().multiply(this.xfo.ori)
      this.localXfo.ori.normalizeInPlace()
    }

    let parentXfo = this.xfo
    for (let i = index + 1; i < joints.length; i++) {
      const joint = joints[i]
      joint.xfo.ori = parentXfo.ori.multiply(joint.localXfo.ori)
      joint.xfo.tr = parentXfo.tr.add(parentXfo.ori.rotateVec3(joint.localXfo.tr))
      parentXfo = joint.xfo
    }
  }

  setClean() {
    for (let key in this.debugLines) this.debugLines[key].doneFrame()
    this.output.setClean(this.xfo)
  }
}

/** An operator for aiming items at targets.
 * @extends Operator
 */
class IKSolver extends Operator {
  /**
   * Create a gears operator.
   * @param {string} name - The name value.
   */
  constructor(name) {
    super(name)

    this.addParameter(new NumberParameter('Iterations', 10))
    this.addInput(new OperatorInput('Base'))
    this.addInput(new OperatorInput('Target'))
    this.__joints = []
    this.enabled = false

    this.debugTree = new TreeItem('IKSolver-debug')
  }

  addJoint(globalXfoParam, axisId = 0) {
    const joint = new IKJoint(this.__joints.length, axisId, this.debugTree)

    const output = this.addOutput(new OperatorOutput('Joint' + this.__joints.length))
    output.setParam(globalXfoParam)
    joint.output = output

    this.__joints.push(joint)
    return joint
  }

  enable() {
    const baseXfo = this.getInput('Base').isConnected() ? this.getInput('Base').getValue() : identityXfo
    this.__joints.forEach((joint, index) => {
      const parentXfo = index > 0 ? this.__joints[index - 1].xfo : baseXfo
      joint.init(parentXfo)
    })
    this.enabled = true
    this.setDirty()
  }

  /**
   * The evaluate method.
   */
  evaluate() {
    if (!this.enabled) {
      this.__joints.forEach(joint => {
        joint.output.setClean(joint.output.getValue()) // until we have an IO output
      })
      return
    }
    const targetXfo = this.getInput('Target').getValue()
    const baseXfo = this.getInput('Base').isConnected() ? this.getInput('Base').getValue() : identityXfo

    const iterations = this.getParameter('Iterations').getValue()
    const numJoints = this.__joints.length

    for (let i = 0; i < iterations; i++) {
      {
        for (let j = numJoints - 1; j >= 0; j--) {
          const joint = this.__joints[j]
          joint.evalCCD(baseXfo, targetXfo, j, this.__joints)
        }
      }
    }

    // Now store the value to the connected Xfo parameter.
    for (let i = 0; i < numJoints; i++) {
      this.__joints[i].setClean()
    }
  }
}

Registry.register('IKSolver', IKSolver)

export { IKSolver }
