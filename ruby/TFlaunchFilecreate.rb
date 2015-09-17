require 'erb'

class TFLaunch

  attr_accessor :name,:x,:y,:z,:roll,:pitch,:yaw,:frameParent,:frameChild

  def initialize name,x,y,z,roll,pitch,yaw,frameParent,frameChild
    @file = File.open("/home/nuc1/ruby/tf_static.launch","w")
    @fichier = @file 
    @name = name
    @x = x
    @y = y
    @z = z
    @roll = roll
    @pitch = pitch
    @yaw = yaw
    @frameParent = frameParent
    @frameChild = frameChild
  end

  def doIt 
  #====== Writing File ======

    tf=self
    string=IO.read "/home/nuc1/ruby/template_tf_static.launch.txt"
    engine = ERB.new(string)
    generated_code= engine.result(binding)

    @fichier.write generated_code
    @file.close
  #=========================
  end

end

tf = TFLaunch.new(ARGV[0],ARGV[1],ARGV[2],ARGV[3],ARGV[4],ARGV[5],ARGV[6],ARGV[7],ARGV[8])

tf.doIt 
