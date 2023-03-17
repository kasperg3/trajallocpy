class Node:
    def __init__(self, angle, distance, edge):
        self.angle = angle
        self.distance = distance
        self.edge = edge
        self.left_child = None
        self.right_child = None
        self.height = 1

    def __str__(self) -> str:
        return f"Node(angle:{self.angle}, distance:{self.distance}, edge:{self.edge})"


class BalancedBinarySearchTree:
    def __init__(self):
        self.root = None

    def insert(self, angle, distance, edge):
        self.root = self._insert_helper(self.root, angle, distance, edge)

    def _insert_helper(self, node: Node, angle, distance, edge):

        if node is None:
            return Node(angle, distance, edge)

        if angle < node.angle or (angle == node.angle and distance < node.distance):
            node.left_child = self._insert_helper(node.left_child, angle, distance, edge)
        else:
            node.right_child = self._insert_helper(node.right_child, angle, distance, edge)

        node.height = 1 + max(self._get_height(node.left_child), self._get_height(node.right_child))

        balance_factor = self._get_balance_factor(node)

        if balance_factor > 1 and angle < node.left_child.angle:
            return self._right_rotate(node)

        if balance_factor > 1 and angle > node.left_child.angle and distance > node.left_child.distance:
            node.left_child = self._left_rotate(node.left_child)
            return self._right_rotate(node)

        if balance_factor < -1 and angle > node.right_child.angle and distance > node.right_child.distance:
            return self._left_rotate(node)

        if balance_factor < -1 and angle < node.right_child.angle and distance < node.right_child.distance:
            node.right_child = self._right_rotate(node.right_child)
            return self._left_rotate(node)

        return node

    def delete(self, angle, distance):
        self.root = self._delete_helper(self.root, angle, distance)

    def _delete_helper(self, node: Node, angle, distance):
        if not node:
            return node
        elif angle < node.angle or (angle == node.angle and distance < node.distance):
            node.left_child = self._delete_helper(node.left_child, angle, distance)
        elif angle > node.angle or (angle == node.angle and distance < node.distance):
            node.right_child = self._delete_helper(node.right_child, angle, distance)
        else:
            if not node.left_child and not node.right_child:
                node = None
            elif not node.left_child:
                node = node.right_child
            elif not node.right_child:
                node = node.left_child
            else:
                min_node = self._find_min(node.right_child)
                node.angle = min_node.angle
                node.distance = min_node.distance
                node.right_child = self._delete_helper(node.right_child, min_node.angle, min_node.distance)

        if not node:
            return node

        node.height = 1 + max(self._get_height(node.left_child), self._get_height(node.right_child))
        balance = self._get_balance_factor(node)

        if balance > 1 and self._get_balance_factor(node.left_child) >= 0:
            return self._right_rotate(node)
        if balance < -1 and self._get_balance_factor(node.right_child) <= 0:
            return self._left_rotate(node)
        if balance > 1 and self._get_balance_factor(node.left_child) < 0:
            node.left_child = self._left_rotate(node.left_child)
            return self._right_rotate(node)
        if balance < -1 and self._get_balance_factor(node.right_child) > 0:
            node.right_child = self._right_rotate(node.right_child)
            return self._left_rotate(node)

        return node

    def _get_height(self, current_node):
        if current_node is None:
            return 0
        return current_node.height

    def _get_balance_factor(self, current_node):
        if current_node is None:
            return 0
        return self._get_height(current_node.left_child) - self._get_height(current_node.right_child)

    def _left_rotate(self, z):
        y = z.right_child
        T2 = y.left_child

        y.left_child = z
        z.right_child = T2

        z.height = 1 + max(self._get_height(z.left_child), self._get_height(z.right_child))
        y.height = 1 + max(self._get_height(y.left_child), self._get_height(y.right_child))

        return y

    def _right_rotate(self, z):
        y = z.left_child
        T3 = y.right_child

        y.right_child = z
        z.left_child = T3

        z.height = 1 + max(self._get_height(z.left_child), self._get_height(z.right_child))
        y.height = 1 + max(self._get_height(y.left_child), self._get_height(y.right_child))

        return y

    def _find_min(self, root: Node):
        if root is None or root.left_child is None:
            return root
        return self._find_min(root.left_child)

    def find_min(self):
        return self._find_min(self.root)

    def pretty_print(self):
        self._print_helper(self.root, "", True)

    def _print_helper(self, current_node, prefix, is_left):
        if current_node is not None:
            self._print_helper(current_node.right_child, prefix + ("│   " if is_left else "    "), False)
            print(prefix + ("└── " if is_left else "┌── ") + str(current_node.angle) + " " + str(current_node.distance))
            self._print_helper(current_node.left_child, prefix + ("    " if is_left else "│   "), True)


class BinarySearchTree:
    def __init__(self):
        self.root = None

    def insert(self, value, edge):
        self.root = self._insert_helper(self.root, value, edge)

    def _insert_helper(self, current_node, value, edge):
        if current_node is None:
            return Node(value, edge)

        if value < current_node.value:
            current_node.left_child = self._insert_helper(current_node.left_child, value, edge)
        else:
            current_node.right_child = self._insert_helper(current_node.right_child, value, edge)

        return current_node

    def pretty_print(self):
        self._print_helper(self.root, "", True)

    def _print_helper(self, current_node, prefix, is_left):
        if current_node is not None:
            self._print_helper(current_node.right_child, prefix + ("│   " if is_left else "    "), False)
            print(prefix + ("└── " if is_left else "┌── ") + str(current_node.value))
            self._print_helper(current_node.left_child, prefix + ("    " if is_left else "│   "), True)


if __name__ == "__main__":
    # create a new balanced binary search tree
    bst = BalancedBinarySearchTree()
    print("Min: ", bst.find_min())
    # insert some values into the tree
    bst.insert(30, 10, "test5")
    bst.insert(10, 20, "test1")
    bst.insert(50, 20, "test2")
    bst.insert(10, 10, "test3")
    bst.insert(40, 20, "test4")
    bst.insert(40, 20, "test4")
    bst.insert(40, 20, "test4")
    bst.insert(40, 20, "test4")
    bst.insert(40, 10, "test4")
    bst.insert(30, 20, "test5")
    bst.insert(20, 20, "test6")
    bst.insert(25, 20, "test7")
    # print out the values in sorted order
    def inorder_traversal(current_node):
        if current_node:
            inorder_traversal(current_node.left_child)
            print(current_node)
            inorder_traversal(current_node.right_child)

    inorder_traversal(bst.root)
    print(bst.pretty_print())
    print("min node: ", bst.find_min())
    bst.delete(40, 10)
    bst.delete(10, 10)
    print(bst.pretty_print())
    print("min node: ", bst.find_min())
