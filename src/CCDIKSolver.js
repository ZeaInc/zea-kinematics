import {
  Quat,
  NumberParameter,
  XfoParameter,
  StructParameter,
  Operator,
  OperatorInput,
  OperatorOutput,
  OperatorOutputMode,
  Registry,
  Xfo,
  Vec3,
  Material,
  GeomItem,
  MathFunctions,
  TreeItem,
  Lines,
  Color
} from '@zeainc/zea-engine'

const X_AXIS = new Vec3(1, 0, 0)
const Y_AXIS = new Vec3(0, 1, 0)
const Z_AXIS = new Vec3(0, 0, 1)
const identityXfo = new Xfo()

class IKJoint {
  constructor(index, axisId = 0, solverDebugTree) {
    this.index = index
    this.axisId = axisId
    this.limits = [-Math.PI, Math.PI]
    this.align = new Quat()

    this.debugTree = new TreeItem('IKJoint' + index)
    solverDebugTree.addChild(this.debugTree)

    this.line = new Lines()
    this.linepositions = this.line.getVertexAttribute('positions')
    this.numDebugSegments = 0
    this.numDebugPoints = 0

    const mat = new Material('debug', 'LinesShader')
    mat.getParameter('BaseColor').setValue(new Color(1, 1, 0))
    mat.getParameter('Overlay').setValue(1)

    const debugGeomItem = new GeomItem('Pointer', this.line, mat)
    this.debugTree.addChild(debugGeomItem)
  }

  addDebugSegment(p0, p1) {
    const pid0 = this.numDebugPoints
    const pid1 = this.numDebugPoints + 1
    this.numDebugSegments++
    this.numDebugPoints += 2
    if (this.line.getNumVertices() < this.numDebugPoints) this.line.setNumVertices(this.numDebugPoints)
    if (this.line.getNumSegments() < this.numDebugSegments) this.line.setNumSegments(this.numDebugSegments)
    this.line.setSegmentVertexIndices(this.numDebugSegments - 1, pid0, pid1)
    this.linepositions.getValueRef(pid0).setFromOther(p0)
    this.linepositions.getValueRef(pid1).setFromOther(p1)
  }

  init(baseXfo, parentJoint, childJoint) {
    this.xfo = this.output.getValue().clone() // until we have an IO output
    const parentXfo = parentJoint ? parentJoint.xfo : baseXfo
    this.bindLocalXfo = parentXfo.inverse().multiply(this.xfo)
    this.localXfo = this.bindLocalXfo.clone()
    if (childJoint) {
      this.backwardsLocal = childJoint.output
        .getValue()
        .inverse()
        .multiply(this.xfo)
    }
    this.forwardLocalTr = this.localXfo.tr
    this.backwardsLocalTr = this.forwardLocalTr.negate()

    this.tipVec = new Vec3()

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

    // const parentVec = this.xfo.tr.subtract(parentXfo.tr).normalize()
    // this.isTwistJoint = Math.abs(this.xfo.ori.rotateVec3(this.axis).dot(parentVec) - 1) < 0.001
    // console.log(this.isTwistJoint)

    // const childXfo =
    // this.xfo = this.output.getValue().clone() // until we have an IO output
    // this.bindLocalXfo = (parentJoint ? parentJoint : baseXfo).xfo.inverse().multiply(this.xfo)
  }

  postInit(joints, jointIndex) {
    const invXfo = this.xfo.inverse()
    const tipXfo = joints[joints.length - 1].xfo
    this.jointVec = invXfo.transformVec3(tipXfo.tr)
    this.jointTipVec = invXfo.transformVec3(tipXfo.tr)
    this.jointTipVecLength = this.jointTipVec.length()

    const axis = this.xfo.ori.rotateVec3(this.axis)
    for (let i = jointIndex + 1; i < joints.length; i++) {
      const nextJoint = joints[i]
      if (nextJoint.xfo.ori.rotateVec3(nextJoint.axis).dot(axis) > 0.001) {
        this.jointVec = invXfo.transformVec3(nextJoint.xfo.tr)
        break
      }
    }
    this.jointVecLength = this.jointVec.length()
  }

  preEval(parentXfo) {
    this.xfo.ori = parentXfo.ori.multiply(this.bindLocalXfo.ori)
    this.xfo.tr = parentXfo.tr.add(parentXfo.ori.rotateVec3(this.bindLocalXfo.tr))

    this.line.emit('geomDataTopologyChanged')
    this.numDebugSegments = 0
    this.numDebugPoints = 0
  }

  evalBackwards(parentJoint, childJoint, isTip, targetXfo, baseXfo, jointToTip) {
    if (isTip) {
      this.xfo.tr = targetXfo.tr.clone()
      this.xfo.ori = targetXfo.ori.clone()

      // this.addDebugSegment(this.xfo.tr, this.xfo.tr.subtract(jointToTip))
      // const jointVec = this.xfo.ori.rotateVec3(this.forwardLocalTr)
      // jointToTip.subtractInPlace(jointVec)
    } else {
      const targetVec = childJoint.xfo.tr.subtract(baseXfo.tr)
      const jointVec = childJoint.xfo.ori.rotateVec3(childJoint.forwardLocalTr)
      const twist = true //childJoint.isTwistJoint
      if (twist) {
        const childTwistVec = childJoint.xfo.ori.rotateVec3(childJoint.axis)
        const vec0 = jointToTip.subtract(childTwistVec.scale(jointToTip.dot(childTwistVec)))
        const vec1 = targetVec.subtract(childTwistVec.scale(targetVec.dot(childTwistVec)))
        this.align.setFrom2Vectors(vec0.normalize(), vec1.normalize())
        this.xfo.ori = this.align.multiply(this.xfo.ori)

        this.addDebugSegment(childJoint.xfo.tr, childJoint.xfo.tr.subtract(jointToTip))
        this.addDebugSegment(childJoint.xfo.tr, childJoint.xfo.tr.subtract(targetVec))
      } else {
        // const childAxis = childJoint.xfo.ori.rotateVec3(childJoint.axis)
        // const vec0 = jointToTip.subtract(jointToTip.scale(jointToTip.dot(childAxis)))
        // const vec1 = targetVec.subtract(targetVec.scale(targetVec.dot(childAxis)))
        // this.align.setFrom2Vectors(vec0.normalize(), vec1.normalize())
        // this.xfo.ori = this.align.multiply(this.xfo.ori)
        // // this.align.setFrom2Vectors(jointToTip.normalize(), targetVec.normalize())
        // // this.xfo.ori = this.align.multiply(this.xfo.ori)
        // const jointVec = this.xfo.ori.rotateVec3(childJoint.forwardLocalTr)
        // ///////////////////////////////
        // // Calc joint0Xfo
        // const jointTargetDist = targetVec.length()
        // const jointLength = childJoint.jointVecLength // jointVec.length()
        // const jointTipToTip = jointToTip.subtract(jointVec)
        // const jointTipToTipDist = jointTipToTip.length()
        // if (jointTipToTipDist + jointLength < jointTargetDist) {
        //   this.align.setFrom2Vectors(jointVec.normalize(), targetVec.normalize())
        //   this.xfo.ori = this.align.multiply(this.xfo.ori)
        // } else {
        //   // Calculate the angle using the rule of cosines.
        //   // cos C	= (a2 + b2 − c2)/2ab
        //   const a = jointLength
        //   const b = jointTargetDist
        //   const c = jointTipToTipDist
        //   const angle = Math.acos((a * a + b * b - c * c) / (2 * a * b))
        //   // console.log(currAngle, angle)
        //   jointVec.normalizeInPlace()
        //   targetVec.normalizeInPlace()
        //   const jointAxis = targetVec.cross(jointVec)
        //   const currAngle = targetVec.angleTo(jointVec)
        //   jointAxis.normalizeInPlace()
        //   this.align.setFromAxisAndAngle(jointAxis, angle - currAngle)
        //   this.xfo.ori = this.align.multiply(this.xfo.ori)
        // }
        ///////////////////////////////
      }
      jointToTip.subtractInPlace(jointVec)

      ///////////////////////
      // Apply joint constraint.
      this.align.setFrom2Vectors(
        this.xfo.ori.rotateVec3(childJoint.axis),
        childJoint.xfo.ori.rotateVec3(childJoint.axis)
      )
      this.xfo.ori = this.align.multiply(this.xfo.ori)
      this.xfo.ori.normalizeInPlace()

      ///////////////////////
      // Apply angle Limits.

      // const currAngle = Math.acos(this.xfo.ori.dot(parentXfo.ori))
      // if (currAngle < childJoint.limits[0] || currAngle > childJoint.limits[1]) {
      //   const deltaAngle =
      //     currAngle < childJoint.limits[0] ? childJoint.limits[0] - currAngle : currAngle - childJoint.limits[1]
      //   this.align.setFromAxisAndAngle(globalAxis, deltaAngle)
      //   this.xfo.ori = this.align.multiply(this.xfo.ori)
      // }

      this.xfo.tr = childJoint.xfo.tr.subtract(this.xfo.ori.rotateVec3(childJoint.forwardLocalTr))
    }
  }

  evalForwards(parentJoint, childJoint, isBase, isTip, baseXfo, targetXfo, jointToTip) {
    let parentXfo
    if (isBase) {
      parentXfo = baseXfo
    } else {
      parentXfo = parentJoint.xfo
    }

    this.xfo.tr = parentXfo.tr.add(parentXfo.ori.rotateVec3(this.forwardLocalTr))
    // this.xfo.tr = parentXfo.tr.add(parentXfo.ori.rotateVec3(this.bindLocalXfo.tr))

    if (isTip) {
      this.xfo.ori = targetXfo.ori
    } else {
      jointToTip.subtractInPlace(parentXfo.ori.rotateVec3(this.forwardLocalTr))

      const targetVec = targetXfo.tr.subtract(this.xfo.tr)

      {
        const twistVec = targetVec.normalize()
        const currjointAxis = parentXfo.ori.rotateVec3(this.axis)
        const goaljointAxis = this.xfo.ori.rotateVec3(this.axis)
        const vec0 = goaljointAxis.subtract(twistVec.scale(goaljointAxis.dot(twistVec)))
        const vec1 = currjointAxis.subtract(twistVec.scale(currjointAxis.dot(twistVec)))
        this.align.setFrom2Vectors(vec0.normalize(), vec1.normalize())
        this.xfo.ori = this.align.multiply(this.xfo.ori)
      }
      // {
      //   this.align.setFrom2Vectors(jointToTip.normalize(), targetVec.normalize())
      //   this.xfo.ori = this.align.multiply(this.xfo.ori)
      // }
      // const twist = true //this.isTwistJoint
      // if (twist) {
      //   const twistVec = this.xfo.ori.rotateVec3(this.axis)
      //   const vec0 = jointToTip.subtract(twistVec.scale(jointToTip.dot(twistVec)))
      //   const vec1 = targetVec.subtract(twistVec.scale(targetVec.dot(twistVec)))
      //   this.align.setFrom2Vectors(vec0.normalize(), vec1.normalize())
      //   this.xfo.ori = this.align.multiply(this.xfo.ori)
      // } else {
      //   const twistVec = this.xfo.ori.rotateVec3(this.axis)
      //   const vec0 = jointToTip.subtract(jointToTip.scale(jointToTip.dot(twistVec)))
      //   const vec1 = targetVec.subtract(targetVec.scale(targetVec.dot(twistVec)))
      //   this.align.setFrom2Vectors(vec0.normalize(), vec1.normalize())
      //   this.xfo.ori = this.align.multiply(this.xfo.ori)

      //   const jointVec = this.xfo.ori.rotateVec3(childJoint.forwardLocalTr)

      //   ///////////////////////////////
      //   // Calc joint0Xfo
      //   const jointTargetDist = targetVec.length()
      //   const jointLength = this.jointVecLength // jointVec.length()
      //   const jointTipToTip = jointToTip.subtract(jointVec)
      //   const jointTipToTipDist = jointTipToTip.length()
      //   if (jointTipToTipDist + jointLength < jointTargetDist) {
      //     this.align.setFrom2Vectors(jointVec.normalize(), targetVec.normalize())
      //     this.xfo.ori = this.align.multiply(this.xfo.ori)
      //   } else {
      //     // Calculate the angle using the rule of cosines.
      //     // cos C	= (a2 + b2 − c2)/2ab
      //     const a = jointLength
      //     const b = jointTargetDist
      //     const c = jointTipToTipDist
      //     const angle = Math.acos((a * a + b * b - c * c) / (2 * a * b))

      //     // console.log(currAngle, angle)

      //     jointVec.normalizeInPlace()
      //     targetVec.normalizeInPlace()
      //     const jointAxis = targetVec.cross(jointVec)
      //     const currAngle = targetVec.angleTo(jointVec)
      //     jointAxis.normalizeInPlace()
      //     this.align.setFromAxisAndAngle(jointAxis, angle - currAngle)
      //     this.xfo.ori = this.align.multiply(this.xfo.ori)
      //   }

      //   ///////////////////////////////
      // }
    }

    ///////////////////////
    // Apply joint constraint.
    this.align.setFrom2Vectors(this.xfo.ori.rotateVec3(this.axis), parentXfo.ori.rotateVec3(this.axis))
    this.xfo.ori = this.align.multiply(this.xfo.ori)
  }

  setClean() {
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
    // this.addParameter(new NumberParameter('Weight', 1))

    // this.jointsParam = this.addParameter(new ListParameter('Joints', CCDIKJointParameter))
    // this.jointsParam.on('elementAdded', event => {
    //   this.addOutput(event.elem.getOutput(), event.index)
    // })
    // this.jointsParam.on('elementRemoved', event => {
    //   this.removeOutput(event.index)
    // })

    this.addInput(new OperatorInput('Base'))
    this.addInput(new OperatorInput('Target'))
    this.__joints = []
    this.enabled = false

    this.debugTree = new TreeItem('IKSolver-debug')
  }

  addJoint(globalXfoParam, axisId = 0) {
    // const output = this.addOutput(new OperatorOutput('Joint', OperatorOutputMode.OP_READ_WRITE))
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
      const parentJoint = index > 0 ? this.__joints[index - 1] : null
      const childJoint = index < this.__joints.length ? this.__joints[index + 1] : null
      joint.init(baseXfo, parentJoint, childJoint)
    })

    for (let i = 0; i < this.__joints.length - 1; i++) {
      const joint = this.__joints[i]
      joint.postInit(this.__joints, i)
    }

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
    // const rootJoint = this.__joints[0]
    const baseXfo = this.getInput('Base').isConnected() ? this.getInput('Base').getValue() : identityXfo

    const numJoints = this.__joints.length
    const tipJoint = this.__joints[numJoints - 1]

    const iterations = 1 //this.getParameter('Iterations').getValue()

    for (let i = 0; i < numJoints; i++) {
      const parentXfo = i > 0 ? this.__joints[i - 1].xfo : baseXfo
      this.__joints[i].preEval(parentXfo)
    }

    for (let i = 0; i < iterations; i++) {
      {
        const jointToTip = tipJoint.xfo.tr.subtract(baseXfo.tr)
        for (let j = numJoints - 1; j >= 0; j--) {
          const joint = this.__joints[j]
          const parentJoint = this.__joints[Math.max(j - 1, 0)]
          const childJoint = this.__joints[Math.min(j + 1, numJoints - 1)]
          const isTip = j > 0 && j == numJoints - 1
          joint.evalBackwards(parentJoint, childJoint, isTip, targetXfo, baseXfo, jointToTip)
        }
      }
      // {
      //   const jointToTip = tipJoint.xfo.tr.subtract(baseXfo.tr)
      //   for (let j = 0; j < numJoints; j++) {
      //     const joint = this.__joints[j]
      //     const parentJoint = this.__joints[Math.max(j - 1, 0)]
      //     const childJoint = this.__joints[Math.min(j + 1, numJoints - 1)]
      //     const isBase = j == 0
      //     const isTip = j > 0 && j == numJoints - 1
      //     joint.evalForwards(parentJoint, childJoint, isBase, isTip, baseXfo, targetXfo, jointToTip)
      //   }
      // }
    }

    // Now store the value to the connected Xfo parameter.
    for (let i = 0; i < numJoints; i++) {
      this.__joints[i].setClean()
    }
  }
}

Registry.register('IKSolver', IKSolver)

export { IKSolver }
