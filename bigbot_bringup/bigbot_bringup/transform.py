import os
import xacro
import subprocess
import re
from tempfile import NamedTemporaryFile
from xml.dom.minidom import parseString
from xml.dom import minidom

''' useage:
from transform import Transformer
a = Transformer('urdf_source')
urdftext = a.geturdf()
sdftext = a.getsdf()
a.savesdf('filename')

A project-structure is expected.
/urdf
/models/<optional other dir>/meshes

The model in the urdf-directory should point to meshes in a models directory

'''

class Transformer:

    def __init__(self, urdf_location):
        doc = xacro.parse(open(urdf_location))
        xacro.process_doc(doc)
        self.urdf = doc
        self.createsdf()

    def update_uri_text(self, uritext):
        pattern = r'model://(?P<urdfpart>[\w,.,/]+)/models/(?P<sdfloc>[\w,.,/]+)'
        searchresult = re.search(pattern, uritext)
        if searchresult is not None:
            return 'model://' + searchresult.groupdict()['sdfloc']
        else:
            return uritext # could not find models-directory

    def newelements_(self, rgba_text):
        text = '<wrap>'
        text+= '<ambient>' + rgba_text + '</ambient>'
        text+= '<diffuse>' + rgba_text + '</diffuse>'
        text+= '<specular>0.01 0.01 0.01 1</specular>'
        text+= '<emissive>0 0 0 1</emissive>'
        text+= '</wrap>'
        return parseString(text).firstChild.childNodes

    def get_colors(self):
        colordef = dict()
        elements = self.urdf.getElementsByTagName('material')
        for element in elements: 
            if element.hasAttribute("name"):
                name = element.getAttribute("name")
                if element.firstChild.hasAttribute("rgba"):
                    colordef[name] = element.firstChild.getAttribute("rgba")
        return colordef

    def name_of_material_in_sdf_(self, element):
        for e in element.childNodes:
            if type(e) == minidom.Element:
                if e.tagName == 'script':
                    for f in e.childNodes:
                        if type(f) == minidom.Element:
                            if f.tagName == 'name':
                                return f.firstChild.nodeValue
        
    def update_colors(self):
        colordef = self.get_colors()
        elements = self.sdf.getElementsByTagName('material')
        for element in elements:
            materialname = self.name_of_material_in_sdf_(element)
            if materialname in colordef:
                newelements = self.newelements_(colordef[materialname])
                for newelement in newelements:
                    element.insertBefore(newelement, element.childNodes[1])

    def update_uris(self):
        elements = self.sdf.getElementsByTagName('uri')
        for element in elements:
            element.firstChild.data = self.update_uri_text(element.firstChild.data)

    def createsdf(self):
        try:
            file = NamedTemporaryFile(delete=False)
            filename = file.name
            file.write(bytes(self.geturdf(), 'ascii'))
            file.close()
            ee = subprocess.run(["gz","sdf","-p",filename], stdout=subprocess.PIPE, universal_newlines=True)
            self.sdf = parseString(ee.stdout)
        finally:
            os.unlink(filename)
        pass
        self.update_colors()
        self.update_uris()

    def getsdf(self):    
        return self.sdf.toprettyxml()
    
    def savesdf(self,filename):
        file = open(filename, 'w')
        file.write(self.getsdf())
        file.close()

    def geturdf(self):
        return self.urdf.toprettyxml()
