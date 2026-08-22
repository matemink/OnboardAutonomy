import QtQuick 2.12
import QtQuick.Controls 2.12

Rectangle {
  anchors.fill: parent
  color: "transparent"

  Rectangle {
    anchors.fill: parent
    anchors.margins: 4
    radius: 10
    color: "#e8172028"
    border.color: "#71808d"
    border.width: 1

    Text {
      anchors.centerIn: parent
      text: "FPS  " + FpsIndicator.framesPerSecond.toFixed(1)
      color: "#f5f8fa"
      font.pixelSize: 13
      font.bold: true
      font.letterSpacing: 0.5
    }
  }
}
