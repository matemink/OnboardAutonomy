import QtQuick 2.12
import QtQuick.Controls 2.12

Rectangle {
  id: root
  anchors.fill: parent
  color: "transparent"

  Rectangle {
    anchors.fill: parent
    anchors.margins: 6
    radius: 14
    color: "#e8172028"
    border.color: "#6fd3e6"
    border.width: 1

    Row {
      anchors.fill: parent
      anchors.margins: 12
      spacing: 12

      Item {
        width: 94
        height: 94
        anchors.verticalCenter: parent.verticalCenter

        Canvas {
          anchors.fill: parent
          onPaint: {
            var context = getContext("2d")
            context.reset()
            context.strokeStyle = "#71808d"
            context.lineWidth = 1
            context.beginPath()
            context.arc(47, 47, 34, 0, Math.PI * 2)
            context.stroke()
          }
        }

        Text {
          text: "N"
          color: "#f3f7fa"
          font.pixelSize: 10
          font.bold: true
          anchors.horizontalCenter: parent.horizontalCenter
          anchors.top: parent.top
        }
        Text {
          text: "E"
          color: "#9aa7b2"
          font.pixelSize: 10
          anchors.right: parent.right
          anchors.verticalCenter: parent.verticalCenter
        }
        Text {
          text: "S"
          color: "#9aa7b2"
          font.pixelSize: 10
          anchors.horizontalCenter: parent.horizontalCenter
          anchors.bottom: parent.bottom
        }
        Text {
          text: "W"
          color: "#9aa7b2"
          font.pixelSize: 10
          anchors.left: parent.left
          anchors.verticalCenter: parent.verticalCenter
        }

        Canvas {
          width: 62
          height: 62
          anchors.centerIn: parent
          visible: WindIndicator.speedMetersPerSecond > 0.0
          rotation: WindIndicator.directionFromDegrees + 180
          transformOrigin: Item.Center
          onPaint: {
            var context = getContext("2d")
            context.reset()
            context.fillStyle = "#62d5e8"
            context.beginPath()
            context.moveTo(31, 2)
            context.lineTo(23, 19)
            context.lineTo(29, 17)
            context.lineTo(29, 52)
            context.lineTo(33, 52)
            context.lineTo(33, 17)
            context.lineTo(39, 19)
            context.closePath()
            context.fill()
          }
        }

        Rectangle {
          width: 8
          height: 8
          radius: 4
          color: "#f3f7fa"
          anchors.centerIn: parent
        }
      }

      Column {
        anchors.verticalCenter: parent.verticalCenter
        spacing: 4

        Text {
          text: "SIMULATION WIND"
          color: "#8e9aa5"
          font.pixelSize: 10
          font.bold: true
          font.letterSpacing: 1.2
        }
        Text {
          text: WindIndicator.speedMetersPerSecond.toFixed(1) + " m/s"
          color: "#f5f8fa"
          font.pixelSize: 24
          font.bold: true
        }
        Text {
          text: WindIndicator.directionLabel
          color: "#62d5e8"
          font.pixelSize: 15
          font.bold: true
        }
        Text {
          text: "turbulence " +
                WindIndicator.turbulenceMetersPerSecond.toFixed(1) +
                " m/s"
          color: "#b7c1c9"
          font.pixelSize: 11
        }
      }
    }
  }
}
