def group_enable(items, state):

    for item in items:

        item.setEnabled(state)

    if state:  # enabled item should be visible
        group_visible(items, state)


def group_visible(items, state):

    for item in items:

        item.setVisible(state)

    if not state:  # invisible item should be disabled
        group_enable(items, state)
