import { Change, UndoRedoManager } from '@zeainc/zea-ux'
import BaseTrack from './BaseTrack.js'

class AddKeyChange extends Change {
  constructor(track, time, value) {
    super(`Add Key to ${track ? track.getName() : 'track'}`)

    if (track != undefined && time != undefined && value != undefined) {
      this.track = track
      this.time = time
      this.value = value
      this.index = track.addKey(this.time, this.value)
    } else {
      super()
    }
  }

  update(value) {
    this.value = value
    this.track.setKeyValue(this.index, this.value)

    this.emit('updated', {
      value: this.value
    })
  }

  undo() {
    this.track.removeKey(this.index)
  }

  redo() {
    this.track.addKey(this.time, this.value)
  }

  /**
   * Serializes `Parameter` instance value as a JSON object, allowing persistence/replication.
   *
   * @param {object} context - The context param.
   * @return {object} The return value.
   */
  toJSON(context) {
    const j = {
      name: this.name,
      trackPath: this.track.getPath(),
      time: this.time
    }

    if (this.value != undefined) {
      if (this.value.toJSON) {
        j.value = this.value.toJSON()
      } else {
        j.value = this.value
      }
    }
    return j
  }

  /**
   * Restores `Parameter` instance's state with the specified JSON object.
   *
   * @param {object} j - The j param.
   * @param {object} context - The context param.
   */
  fromJSON(j, context) {
    const track = context.appData.scene.getRoot().resolvePath(j.trackPath, 1)
    if (!track || !(track instanceof BaseTrack)) {
      console.warn('resolvePath is unable to resolve', j.trackPath)
      return
    }
    this.name = `Add Key to ${track.getName()}`
    this.track = track
    const key = this.track.loadKeyJSON(j)
    this.time = key.time
    this.value = key.value
    this.index = this.track.addKey(key.time, key.value)
  }

  /**
   * Updates the state of an existing identified `Parameter` through replication.
   *
   * @param {object} j - The j param.
   */
  changeFromJSON(j) {
    if (!this.track) return
    if (j.value) {
      this.value = j.value
      this.track.setKeyValue(this.index, this.value)
    }
  }
}

UndoRedoManager.registerChange('AddKeyChange', AddKeyChange)

export default AddKeyChange
export { AddKeyChange }
