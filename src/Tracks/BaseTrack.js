import { EventEmitter, Vec2, Registry } from '@zeainc/zea-engine'

/** Class representing a gear parameter.
 * @extends BaseTrack
 */
class BaseTrack extends EventEmitter {
  constructor(name) {
    super()
    this.name = name
    this.keys = []
    this.__sampleCache = {}

    this.__currChange = null
    this.__secondaryChange = null
    this.__secondaryChangeTime = -1
  }

  getName() {
    return this.name
  }

  getNumKeys() {
    return this.keys.length
  }

  getKeyTime(index) {
    return this.keys[index].time
  }

  getKeyValue(index) {
    return this.keys[index].value
  }

  setKeyValue(index, value) {
    this.keys[index].value = value
    this.emit('keyChanged', { index })
  }

  getTimeRange() {
    if (this.keys.length == 0) {
      return new Vec2(Number.NaN, Number.NaN)
    }
    const numKeys = this.keys.length
    return new Vec2(this.keys[0].time, this.keys[numKeys - 1].time)
  }

  addKey(time, value) {
    let index
    const numKeys = this.keys.length
    if (this.keys.length == 0 || time < this.keys[0].time) {
      this.keys.splice(0, 0, { time, value })
      index = 0
    } else {
      if (time > this.keys[numKeys - 1].time) {
        this.keys.push({ time, value })
        index = numKeys
      } else {
        // Find the first key after the specified time value
        for (let i = 1; i < numKeys; i++) {
          const key = this.keys[i]
          if (key.time > time) {
            this.keys.splice(i, 0, { time, value })
            index = i
            break
          }
        }
      }
    }

    this.emit('keysIndicesChanged', { range: [index, numKeys], delta: 1 })
    this.emit('keyAdded', { index })
    return index
  }

  removeKey(index) {
    // const undoRedoManager = UndoRedoManager.getInstance()
    // const change = undoRedoManager.getCurrentChange()
    // if (change) {
    //   if (this.__currChange != change || this.__secondaryChangeTime != time) {
    //     this.__currChange = change
    //     this.__secondaryChangeTime = time
    //   }
    // }
    this.keys.splice(index, 1)
    const numKeys = this.keys.length
    this.emit('keysIndicesChanged', { range: [index, numKeys], delta: -1 })
    this.emit('keyRemoved', { index })
  }

  findKeyAndLerp(time) {
    if (this.keys.length == 0) {
      return {
        keyIndex: -1,
        lerp: 0
      }
    }
    if (time <= this.keys[0].time) {
      return {
        keyIndex: 0,
        lerp: 0
      }
    }
    const numKeys = this.keys.length
    if (time >= this.keys[numKeys - 1].time) {
      return {
        keyIndex: numKeys - 1,
        lerp: 0
      }
    }
    // Find the first key after the specified time value
    for (let i = 1; i < numKeys; i++) {
      const key = this.keys[i]
      if (key.time > time) {
        const prevKey = this.keys[i - 1]
        const delta = key.time - prevKey.time
        return {
          keyIndex: i - 1,
          lerp: (time - prevKey.time) / delta
        }
      }
    }
  }

  evaluate(time) {
    const keyAndLerp = this.findKeyAndLerp(time)
  }

  setValue(time, value) {
    // const undoRedoManager = UndoRedoManager.getInstance()
    // const change = undoRedoManager.getCurrentChange()
    // if (change) {
    //   if (this.__currChange != change || this.__secondaryChangeTime != time) {
    //     this.__currChange = change
    //     this.__secondaryChangeTime = time

    //     const keyAndLerp = this.findKeyAndLerp(time)
    //     if (keyAndLerp.lerp > 0.0) {
    //       this.__secondaryChange = new AddKeyChange(this, time, value)
    //       this.__currChange.secondaryChanges.push(this.__secondaryChange)
    //     } else {
    //       this.__secondaryChange = new KeyChange(this, keyAndLerp.keyIndex, value)
    //       this.__currChange.secondaryChanges.push(this.__secondaryChange)
    //     }
    //   } else {
    //     this.__secondaryChange.update(value)
    //   }
    // }

    const keyAndLerp = this.findKeyAndLerp(time)
    if (keyAndLerp.lerp > 0.0) {
      this.addKey(time, value)
    } else {
      this.setKeyValue(keyAndLerp.keyIndex, value)
    }
  }

  // ////////////////////////////////////////
  // Persistence

  /**
   * Encodes the current object as a json object.
   *
   * @param {object} context - The context value.
   * @return {object} - Returns the json object.
   */
  toJSON(context) {
    const j = {}
    j.name = this.name
    j.type = Registry.getBlueprintName(this)
    j.keys = this.keys.map(key => {
      return { time: key.time, value: key.value.toJSON ? key.value.toJSON() : key.value }
    })
    return j
  }

  /**
   * Decodes a json object for this type.
   *
   * @param {object} j - The json object this item must decode.
   * @param {object} context - The context value.
   */
  fromJSON(j, context) {
    this.__name = j.name
    this.keys = j.keys.map(keyJson => this.loadKeyJSON(keyJson))
    this.emit('loaded')
  }

  loadKeyJSON(json) {
    const key = {
      time: json.time,
      value: json.value
    }
    return key
  }
}

export { BaseTrack }
