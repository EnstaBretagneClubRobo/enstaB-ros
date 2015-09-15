require 'erb'

class MapLaunch

  attr_accessor :width,:poseX,:poseY

  def initialize width,poseX,poseY
    @file = File.open("/home/nuc1/ruby/mapping_default.launch","w")
    @fichier = @file 
    @text=""
    @width = width
    @poseX = poseX
    @poseY = poseY
  end

  def doIt 
  #====== Writing File ======

    maplaunch=self
    string=IO.read "template_mapping_default_launch.text"
    engine = ERB.new(string)
    generated_code= engine.result(binding)

    @fichier.write generated_code
    @file.close
  #=========================
  end

end

map = MapLaunch.new(ARGV[0],ARGV[1],ARGV[2])

map.doIt 
