import { Change, UndoRedoManager } from '@zeainc/zea-ux'

class KeyChange extends Change {
  constructor(track, index, value, time) {
    super()
    if (track != undefined && index != undefined && value != undefined) {
      this.track = track
      this.index = index
      this.prevValue = this.track.getKeyValue(this.index)
      this.newValue = value
      if (time != undefined) {
        this.newTime = time
        this.track.setKeyTimeAndValue(this.index, this.newTime, this.newValue)
      } else {
        this.newTime = this.track.getKeyTime(this.index)
        this.track.setKeyValue(this.index, this.newValue)
      }
    } else {
      super()
    }
  }

  update(value, time) {
    this.newValue = value
    if (time != undefined) {
      this.newTime = time
      this.track.setKeyTimeAndValue(this.index, this.newTime, this.newValue)
    } else {
      this.track.setKeyValue(this.index, this.newValue)
    }
    this.emit('updated', {
      newValue: this.newValue.toJSON ? this.newValue.toJSON() : this.newValue,
      newTime: this.newTime
    })
  }

  undo() {
    this.track.setKeyValue(this.index, this.prevValue)
  }

  redo() {
    this.track.setKeyValue(this.index, this.newValue)
  }

  toJSON(context) {
    const j = {
      name: this.name,
      trackPath: this.track.getPath(),
      newTime: this.newTime,
      newValue: this.newValue
    }
    return j
  }

  fromJSON(j, context) {
    const track = context.appData.scene.getRoot().resolvePath(j.trackPath, 1)
    if (!track || !(track instanceof BaseTrack)) {
      console.warn('resolvePath is unable to resolve', j.trackPath)
      return
    }
    this.name = `Add Key to ${track.getName()}`
    this.track = track
    const key = this.track.loadKeyJSON(j)
    this.index = key.index
    this.newTime = key.time
    this.newValue = key.newValue
    this.track.setKeyTimeAndValue(this.index, this.newTime, this.newValue)
  }

  changeFromJSON(j) {
    if (!this.track) return
    const key = this.track.loadKeyJSON(j)
    this.newTime = key.time
    this.newValue = key.newValue
    this.track.setKeyTimeAndValue(this.index, this.newTime, this.newValue)
  }
}

UndoRedoManager.registerChange('KeyChange', KeyChange)

export default KeyChange
export { KeyChange }
