#!/usr/bin/env python

import rospy
import importlib


class InstructionsHelperException(Exception):
    pass


class GuiInstruction(object):

    def __init__(self):

        self.learn = None
        self.run = None
        self.vis = None


class BrainInstruction(object):

    def __init__(self):

        pass


class Instruction(object):

    def __init__(self):

        self.gui = GuiInstruction()
        self.brain = BrainInstruction()


class InstructionsProperties(object):

    def __init__(self):

        self.using_object = frozenset()
        self.using_pose = frozenset()
        self.using_polygon = frozenset()
        self.pick = frozenset()
        self.place = frozenset()
        self.ref_to_pick = frozenset()
        self.runnable_during_learning = frozenset()


class InstructionsHelper(object):

    def __init__(self):

        self.packages = frozenset()
        self._instructions = {}
        self.properties = InstructionsProperties()

        self._load()

    def __getitem__(self, key):

        return self._instructions[key]

    def _load(self):

        while not rospy.is_shutdown():
            try:
                instructions = rospy.get_param("/art/instructions")
                break
            except KeyError:
                rospy.loginfo("Waiting for /art/instructions param...")
                rospy.sleep(1.0)

        if rospy.is_shutdown():
            raise InstructionsHelperException("Shutdown.")

        packages = set()

        try:

            for k, v in instructions["instructions"].iteritems():
                packages.add(v["gui"]["package"])

        except KeyError as e:

            rospy.logerr(str(e))
            raise InstructionsHelperException("Missing key - invalid configuration.")

        self.packages = frozenset(packages)

        for k, v in instructions["instructions"].iteritems():

            ins = Instruction()

            try:

                mod = importlib.import_module(v["gui"]["package"] + ".gui")

                try:
                    ins.gui.learn = getattr(mod, v["gui"]["learn"])
                    ins.gui.run = getattr(mod, v["gui"]["run"])
                except KeyError as e:

                    rospy.logerr(str(e))
                    raise InstructionsHelperException("Invalid instruction.")

                # visualization class is not mandatory
                if "vis" in v["gui"]:
                    ins.gui.vis = getattr(mod, v["gui"]["vis"])

            except Exception as e:

                rospy.logerr("Failed to import gui instruction: " + str(e))
                raise InstructionsHelperException("Failed to import gui instruction: " + k)

            # TODO brain instructions
            # ins.brain.xyz = getattr(mod, v["brain"]["xyz"])

            self._instructions[k] = ins

        for prop in self.properties.__dict__.keys():

            try:
                self.properties.__dict__[prop] = frozenset(instructions[prop])
            except KeyError:
                rospy.logwarn("Key " + prop + " not defined.")

    def known_instructions(self):

        return self._instructions.keys()

    def requires_learning(self, ins):

        return ins in self.properties.using_object | self.properties.using_polygon | self.properties.using_pose
