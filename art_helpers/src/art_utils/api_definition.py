import rospy


class ArtAPI():

    def __init__(self):

        self.groups = []

        for gr in rospy.get_param("/art/api"):

            tmp = []

            if "topics" in gr:
                for topic in gr["topics"]:

                    self._fix_desc(topic)
                    tmp.append(
                        Topic(topic["name"],  topic["type"], topic["description"]))

            if "services" in gr:
                for service in gr["services"]:

                    self._fix_desc(service)
                    tmp.append(
                        Service(service["name"],  service["type"], service["description"]))

            if "actions" in gr:
                for action in gr["actions"]:

                    self._fix_desc(action)
                    tmp.append(
                        Action(action["name"],  action["type"], action["description"]))

            self._fix_desc(gr)
            self.groups.append(APIGroup(gr["name"], tmp, gr["description"]))

    def _fix_desc(self, it):

        if "description" not in it:
            it["description"] = ""


class APIGroup():

    def __init__(self, name, api, desc=""):

        self.name = name
        self.desc = desc
        self.api = api


class APIItem():

    def __init__(self, name, type, desc=""):

        self.name = name
        self.type = type
        self.desc = desc


class Topic(APIItem):

    pass


class Service(APIItem):

    pass


class Action(APIItem):

    pass
