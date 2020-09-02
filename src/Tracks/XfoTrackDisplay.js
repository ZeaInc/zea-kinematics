import {
  Color,
  GeomItem,
  Lines,
  Cuboid,
  Points,
  Material,
  Operator,
  OperatorOutput,
  OperatorOutputMode
} from '@zeainc/zea-engine'
import { BaseTrack } from './BaseTrack'

/** An operator for aiming items at targets.
 * @extends Operator
 */
class KeyDisplayOperator extends Operator {
  /**
   * Create a gears operator.
   * @param {BaseTrack} track - The track value.
   * @param {number} keyIndex - The index of the key in the track
   */
  constructor(track, keyIndex) {
    super(name)

    this.addOutput(new OperatorOutput('KeyLocal', OperatorOutputMode.OP_WRITE))

    this.track = track
    this.keyIndex = keyIndex
    this.track.on('keyChanged', event => {
      if (event.index == this.keyIndex) this.setDirty()
    })
    this.track.on('keysIndicesChanged', event => {
      const { range, delta } = event
      if (this.keyIndex > range[0] && this.keyIndex < range[1]) {
        // this.keyIndex += delta
        this.setDirty()
      }
    })
    this.track.on('keyRemoved', event => {
      const { index } = event
      if (this.keyIndex >= index) {
        this.setDirty()
      }
    })
    this.track.on('keyAdded', event => {
      const { index } = event
      if (this.keyIndex >= index) {
        this.setDirty()
      }
    })
  }

  backPropagateValue(value) {
    this.track.setKeyValue(this.keyIndex, value)
  }

  /**
   * The evaluate method.
   */
  evaluate() {
    this.getOutputByIndex(0).setClean(this.track.getKeyValue(this.keyIndex))
  }
}

/** An operator for aiming items at targets.
 * @extends Operator
 */
class XfoTrackDisplay extends GeomItem {
  /**
   * Create a TrackDisplay operator.
   * @param {string} name - The name value.
   * @param {BaseTrack} track - The track to display.
   */
  constructor(track) {
    super(track.getName())

    this.track = track

    this.getParameter('Geometry').setValue(new Lines())

    const linesMat = new Material('trackLine', 'FlatSurfaceShader')
    linesMat.getParameter('BaseColor').setValue(new Color(0.3, 0.3, 0.3))
    this.getParameter('Material').setValue(linesMat)

    const dotsMat = new Material('trackDots', 'PointsShader')
    dotsMat.getParameter('BaseColor').setValue(new Color(0.75, 0.75, 0.75))
    this.dotsItem = new GeomItem('dots', new Points(), dotsMat)
    this.addChild(this.dotsItem)

    try {
      this.__keyMat = new Material('trackLine', 'HandleShader')
      this.__keyMat.getParameter('maintainScreenSize').setValue(1)
      this.__keyCube = new Cuboid(0.004, 0.004, 0.004)
    } catch (error) {
      this.__keyMat = new Material('trackLine', 'SimpleSurfaceShader')
      this.__keyCube = new Cuboid(0.01, 0.01, 0.01)
    }

    this.__keys = []
    this.__updatePath()
    this.__displayKeys()

    this.track.on('keyAdded', event => {
      this.__displayKeys()
      this.__updatePath()
    })
    this.track.on('keyRemoved', event => {
      const { index } = event
      const handle = this.__keys.pop()
      this.removeChild(this.getChildIndex(handle))
      this.__displayKeys()
      this.__updatePath()
    })
    this.track.on('keyChanged', event => {
      this.__updatePath()
    })
    this.track.on('loaded', event => {
      this.__updatePath()
      this.__displayKeys()
    })
  }

  __displayKeys() {
    const displayKey = index => {
      if (!this.__keys[index]) {
        const handle = new GeomItem('key' + index, this.__keyCube, this.__keyMat)
        this.addChild(handle)
        const keyDisplay = new KeyDisplayOperator(this.track, index)
        keyDisplay.getOutput('KeyLocal').setParam(handle.getParameter('LocalXfo'))
        this.__keys.push(handle)
      }
    }

    const numKeys = this.track.getNumKeys()
    for (let i = 0; i < numKeys; i++) {
      displayKey(i)
    }
  }

  __updatePath() {
    const trackLines = this.getParameter('Geometry').getValue()
    const trackDots = this.dotsItem.getParameter('Geometry').getValue()

    const timeRange = this.track.getTimeRange()
    if (Number.isNaN(timeRange.x) || Number.isNaN(timeRange.y)) return

    const numSamples = Math.round((timeRange.y - timeRange.x) / 50) // Display at 50 samples per second
    if (numSamples == 0) return

    trackLines.setNumVertices(numSamples + 1)
    trackLines.setNumSegments(numSamples)

    trackDots.setNumVertices(numSamples + 1)
    const linePositions = trackLines.getVertexAttribute('positions')
    const dotPositions = trackDots.getVertexAttribute('positions')
    for (let i = 0; i <= numSamples; i++) {
      if (i < numSamples) trackLines.setSegmentVertexIndices(i, i, i + 1)
      const time = timeRange.x + (timeRange.y - timeRange.x) * (i / numSamples)
      const xfo = this.track.evaluate(time)
      linePositions.getValueRef(i).setFromOther(xfo.tr)
      dotPositions.getValueRef(i).setFromOther(xfo.tr)
    }

    trackDots.setBoundingBoxDirty()
    trackDots.emit('geomDataTopologyChanged', {})

    trackLines.setBoundingBoxDirty()
    trackLines.emit('geomDataTopologyChanged', {})
  }
}

export { XfoTrackDisplay }
