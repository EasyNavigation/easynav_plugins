# A single directed segment between two poses -- the wire form of
# easynav::RouteSegment (see routes_map.hpp), minus its UI-editor-only
# `edit_mode` field.

string id
geometry_msgs/Pose start
geometry_msgs/Pose end
