# ShengNovatekTouch

NT36532 touch protocol and sheng sensor configuration.

The tracker confirms tip-down on the second scan and reports the first
position, including at the bottom edge. A new contact requires at least two
sensor nodes above the noise floor. The panel reference updates after eight
complete scans without a finger or palm. During a pinch with distinct peaks,
each established contact keeps its ID. A partial dropout can retain the last
position for up to three scans.

The device repository selects this component with its board inputs.
Build requirements and scope are documented in the [UniXNU site](https://unixnu.com/drivers/sheng-novatek-touch)
and [component catalog](https://unixnu.com/reference/components).
