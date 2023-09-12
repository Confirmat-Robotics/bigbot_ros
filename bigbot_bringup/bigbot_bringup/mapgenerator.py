import os
from tempfile import NamedTemporaryFile

class MapGenerator:

  def __init__(self, width, height, res = 0.05):
    ''' This function will generate a pgm file and a yaml file in the temporary directory 
        (and will destroy them after use)
    '''
    start = "P2\n%d %d\n255\n" % (width, height) 
    cnt = 0
    totw = 50    
    for i in range(width*height):
      cnt += 1
      start += "255 "
      if cnt > totw:
      	start += "\n"
      	cnt=0
    #self.f = open(filename, "wb")
    f = NamedTemporaryFile(suffix=".pgm", prefix="map_", delete=False)
    self.pgmfilename = f.name
    f.write(start.encode('utf-8'))
    f.close()
    yaml = "image: %s\n" % self.pgmfilename
    yaml += "resolution: %1.5f\n" % (res)
    yaml += "origin: [-%1.5f, -%1.5f, 0.000000]\n" % (width*res/2.0, height*res/2.0)
    yaml += "negate: 0\n"
    yaml += "occupied_thresh: 0.65\n"
    yaml += "free_thresh: 0.196\n"
    g = NamedTemporaryFile(mode="w", suffix=".yaml", prefix="map_", delete=False)
    self.yamlfilename = g.name
    g.write(yaml)
    g.close()
    
  def getpgmfile(self):
    return self.pgmfilename
    
  def getyamlfile(self):
    return self.yamlfilename
    
  def __del__(self):
    os.unlink(self.pgmfilename)
    os.unlink(self.yamlfilename)

