import { Operator, OperatorInput, OperatorOutput, OperatorOutputMode } from '@zeainc/zea-engine'
import { UndoRedoManager } from '@zeainc/zea-ux'
import AddKeyChange from './AddKeyChange.js'
import KeyChange from './KeyChange.js'

/** An operator for aiming items at targets.
 * @extends Operator
 */
class TrackSampler extends Operator {
  /**
   * Create a TrackSampler operator.
   * @param {string} name - The name value.
   */
  constructor(name, track) {
    super(name)

    this.track = track
    this.track.on('keyAdded', this.setDirty.bind(this))
    this.track.on('keyRemoved', this.setDirty.bind(this))
    this.track.on('keyChanged', this.setDirty.bind(this))

    this.addInput(new OperatorInput('Time'))
    this.addOutput(new OperatorOutput('Output', OperatorOutputMode.OP_WRITE))

    this.__currChange = null
    this.__secondaryChange = null
    this.__secondaryChangeTime = -1
  }

  /**
   * @param {Xfo} value - The value param.
   * @return {any} - The modified value.
   */
  backPropagateValue(value) {
    const time = this.getInput('Time').getValue()
    // this.track.setValue(time, value)

    const undoRedoManager = UndoRedoManager.getInstance()
    const change = undoRedoManager.getCurrentChange()
    if (change) {
      if (this.__currChange != change || this.__secondaryChangeTime != time) {
        this.__currChange = change
        this.__secondaryChangeTime = time

        const keyAndLerp = this.track.findKeyAndLerp(time)
        if (
          keyAndLerp.keyIndex == -1 ||
          keyAndLerp.lerp > 0.0 ||
          (keyAndLerp.keyIndex == this.track.getNumKeys() - 1 && this.track.getKeyTime(keyAndLerp.keyIndex) != time)
        ) {
          this.__secondaryChange = new AddKeyChange(this.track, time, value)
          this.__currChange.secondaryChanges.push(this.__secondaryChange)
        } else {
          this.__secondaryChange = new KeyChange(this.track, keyAndLerp.keyIndex, value)
          this.__currChange.secondaryChanges.push(this.__secondaryChange)
        }
      } else {
        this.__secondaryChange.update(value)
      }
    }

    return value
  }

  /**
   * The evaluate method.
   */
  evaluate() {
    const output = this.getOutputByIndex(0)
    if (this.track.getNumKeys() == 0) {
      output.setClean(output.getValue())
    } else {
      const time = this.getInput('Time').getValue()

      const xfo = this.track.evaluate(time)
      output.setClean(xfo)
    }
  }
}

export { TrackSampler }
