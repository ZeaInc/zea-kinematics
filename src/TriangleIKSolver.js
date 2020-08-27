import { Quat, Operator, OperatorInput, OperatorOutput, OperatorOutputMode, Registry } from '@zeainc/zea-engine'

/** An operator for aiming items at targets.
 * @extends Operator
 */
class TriangleIKSolver extends Operator {
  /**
   * Create a gears operator.
   * @param {string} name - The name value.
   */
  constructor(name) {
    super(name)
    // this.addParameter(
    //   new MultiChoiceParameter('Axis', 0, ['+X Axis', '-X Axis', '+Y Axis', '-Y Axis', '+Z Axis', '-Z Axis'])
    // )
    this.addInput(new OperatorInput('Target'))
    this.addOutput(new OperatorOutput('Joint0', OperatorOutputMode.OP_READ_WRITE))
    this.addOutput(new OperatorOutput('Joint1', OperatorOutputMode.OP_READ_WRITE))
    this.align = new Quat()
    this.enabled = false
  }

  enable() {
    const targetXfo = this.getInput('Target').getValue()
    const joint0Xfo = this.getOutput('Joint0').getValue()
    const joint1Xfo = this.getOutput('Joint1').getValue()
    this.joint1Offset = joint0Xfo.inverse().multiply(joint1Xfo).tr
    this.joint1TargetOffset = joint1Xfo.inverse().multiply(targetXfo).tr
    this.joint1TargetOffset.normalizeInPlace()
    this.joint0Length = joint1Xfo.tr.distanceTo(joint0Xfo.tr)
    this.joint1Length = targetXfo.tr.distanceTo(joint1Xfo.tr)
    this.setDirty()
    this.enabled = true
  }

  /**
   * The evaluate method.
   */
  evaluate() {
    const targetXfo = this.getInput('Target').getValue()
    const joint0Output = this.getOutput('Joint0')
    const joint1Output = this.getOutput('Joint1')
    const joint0Xfo = joint0Output.getValue()
    const joint1Xfo = joint1Output.getValue()

    ///////////////////////////////
    // Calc joint0Xfo
    const joint0TargetVec = targetXfo.tr.subtract(joint0Xfo.tr)
    const joint0TargetDist = joint0TargetVec.length()
    const joint01Vec = joint0Xfo.ori.rotateVec3(this.joint1Offset)

    // Calculate the angle using the rule of cosines.
    // cos C	= (a2 + b2 − c2)/2ab
    const a = this.joint0Length
    const b = joint0TargetDist
    const c = this.joint1Length
    const angle = Math.acos((a * a + b * b - c * c) / (2 * a * b))

    // console.log(currAngle, angle)

    joint01Vec.normalizeInPlace()
    joint0TargetVec.normalizeInPlace()

    const Joint0Axis = joint0TargetVec.cross(joint01Vec)
    const currAngle = joint0TargetVec.angleTo(joint01Vec)
    Joint0Axis.normalizeInPlace()

    this.align.setFromAxisAndAngle(Joint0Axis, angle - currAngle)
    joint0Xfo.ori = this.align.multiply(joint0Xfo.ori)

    ///////////////////////////////
    // Calc joint1Xfo
    joint1Xfo.tr = joint0Xfo.transformVec3(this.joint1Offset)

    const joint1TargetVec = targetXfo.tr.subtract(joint1Xfo.tr)
    joint1TargetVec.normalizeInPlace()
    this.align.setFrom2Vectors(joint1Xfo.ori.rotateVec3(this.joint1TargetOffset), joint1TargetVec)
    joint1Xfo.ori = this.align.multiply(joint1Xfo.ori)

    ///////////////////////////////
    // Done
    joint0Output.setClean(joint0Xfo)
    joint1Output.setClean(joint1Xfo)
  }
}

Registry.register('TriangleIKSolver', TriangleIKSolver)

export { TriangleIKSolver }
