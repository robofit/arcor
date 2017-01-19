import matplotlib.path as mplPath
import numpy as np


class ArtBrainUtils(object):

    @staticmethod
    def get_pick_obj_id(instruction, objects):
        if instruction.spec == instruction.MANIP_ID:
            obj_id = instruction.object
        elif instruction.spec == instruction.MANIP_TYPE:

            pick_polygon = []
            pol = None

            for point in instruction.pick_polygon.polygon.points:  # TODO check frame_id and transform to table frame?
                pick_polygon.append([point.x,  point.y])
            pick_polygon.append([0,  0])

            if len(pick_polygon) > 0:
                pol = mplPath.Path(np.array(pick_polygon), closed=True)

            # shuffle the array to not get the same object each time
            # random.shuffle(self.objects.instances)

            print objects.instances

            for obj in objects.instances:

                if pol is None:

                    # if no pick polygon is specified - let's take the first object of that type
                    if obj.object_type == instruction.object:
                        obj_id = obj.object_id
                        break

                else:

                    # test if some object is in polygon and take the first one
                    if pol.contains_point([obj.pose.position.x,  obj.pose.position.y]):
                        obj_id = obj.object_id
                        print('Selected object: ' + obj_id)
                        break

            else:
                if pol is not None:
                    print('No object in the specified polygon')
                    print pol
                return None
        else:
            print "strange instruction.spec: " + str(instruction.spec)
            return None
        return obj_id

    @staticmethod
    def get_place_pose(instruction):

        if instruction.spec == instruction.MANIP_ID:
            pose = instruction.place_pose
        elif instruction.spec == instruction.MANIP_TYPE:
            # pose = None
            pose = instruction.place_pose
            # TODO: how to get free position inside polygon? some perception node?
        else:
            return None
        return pose
