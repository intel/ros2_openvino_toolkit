#!/usr/bin/python

from __future__ import unicode_literals  # at top of module
from __future__ import division, print_function, with_statement
from PyQt5.QtGui import QPainter,QPen,QBrush,QColor
from PyQt5.QtCore import QRect

class TreeNode(object):
    """The basic node of tree structure"""

    def __init__(self, name, parent=None):
        super(TreeNode, self).__init__()
        self.name = name
        self.parent = parent
        self.child = {}

    def __repr__(self) :
        return 'TreeNode(%s)' % self.name


    def __contains__(self, item):
        return item in self.child


    def __len__(self):
        """return number of children node"""
        return len(self.child)

    def __bool__(self, item):
        """always return True for exist node"""
        return True
    
    @property
    def path(self):
        """return path string (from root to current node)"""
        if self.parent:
            return '%s %s' % (self.parent.path.strip(), self.name)
        else:
            return self.name

    def get_child(self, name, defval=None):
        """get a child node of current node"""
        return self.child.get(name, defval)

    def add_child(self, name, obj=None):
        """add a child node to current node"""
        if obj and not isinstance(obj, TreeNode):
            raise ValueError('TreeNode only add another TreeNode obj as child')
        if obj is None:
            obj = TreeNode(name)
        obj.parent = self
        self.child[name] = obj
        return obj
    def del_child(self, name):
        """remove a child node from current node"""
        if name in self.child:
            del self.child[name]

    def find_child(self, path, create=False):
        """find child node by path/name, return None if not found"""
        # convert path to a list if input is a string
        path = path if isinstance(path, list) else path.split()
        cur = self
        obj = None
        for sub in path:
            # search
            obj = cur.get_child(sub)
            if obj is None and create:
                # create new node if need
                obj = cur.add_child(sub)
            # check if search done
            if obj is None:
                break
            cur = obj
        return obj
    def find_node(self,name):
        for name,obj in self.items():
            if name == name:
                return obj
            return obj.find_node(name)
    def items(self):
        return self.child.items()
    
    def dump(self, indent=0):
        """dump tree to string"""
        tab = '    '*(indent-1) + ' |- ' if indent > 0 else ''
        print('%s%s' % (tab, self.name))
        for name, obj in self.items():
            obj.dump(indent+1)

    def dump_graphics(self,convas,name,state,x,y,w,h):
        rect_width = 160
        rect_height = 30
        p = QPainter()
        if state == '2': 
            color = QColor(0, 200, 0) 
           
        elif state == '3':
            color = QColor(255,255,0)
        else:
             color = QColor(200, 0, 0)
        draw_x =   x + (rect_width + 40)
        for i,(name,obj) in enumerate(self.items()):
            p.begin(convas)
            p.setPen(QPen())
            p.setBrush(QBrush(color))
            draw_y = y + i * (rect_height + 10)
            rect = QRect(draw_x ,draw_y,rect_width,rect_height) 
            p.drawRect(rect) 
            p.drawText(draw_x+2,draw_y + rect_height/2,name)  
            if self.name != 'root':
                p.drawLine(x+rect_width,y+rect_height/2,draw_x,draw_y+rect_height/2) 
            p.end()

            obj.dump_graphics(convas,name,state,draw_x,draw_y,w,h)


    def depth(self,depth = 0):
        if not self.items():
            return depth
        max_depth = 0
        for name,obj in self.items():
            
            max_depth = max(max_depth, obj.depth(depth+1))
        return max_depth
