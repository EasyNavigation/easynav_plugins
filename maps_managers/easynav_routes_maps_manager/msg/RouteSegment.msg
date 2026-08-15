# A single directed segment between two poses -- the wire form of
# easynav::RouteSegment (see RoutesMapsManager.hpp), minus its
# UI-editor-only `edit_mode` field.

string id
geometry_msgs/Pose start
geometry_msgs/Pose end
