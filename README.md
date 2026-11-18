# ShengNovatekTouch

NT36532 touch protocol and sheng sensor configuration.

The tracker confirms tip-down on the second scan and reports the first position,
including at the bottom edge. Starting a new contact requires at least two
coupled nodes above the noise floor. The reference follows the panel only
while nothing touches it: in scans without a finger or palm, from the eighth
such scan on. A hand also changes nodes far from it, and a reference that
follows them during the touch reads the change back as contacts after the
lift. A warm node that is no contact does not stop the reference. When a pinch
has distinct peaks, each established contact keeps its ID. During a partial
dropout, the tracker holds the last position for up to three scans. A quiet
matrix releases the contact.

The device repository selects this component with its board inputs.
Build requirements and scope are documented in the [UniXNU site](https://unixnu.com/drivers/sheng-novatek-touch)
and [component catalog](https://unixnu.com/reference/components).
