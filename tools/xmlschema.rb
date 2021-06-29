#!/usr/bin/env ruby

require "rexml/document"
require "optparse"
require "set"

$path = nil

#################################################
# \brief A not very elegant way to convert to schema types
def xsdType(_type)
  if _type == "unsigned int"
    return "unsignedInt"
  elsif _type == "unsigned long"
    return "unsignedLog"
  elsif _type == "bool"
    return "boolean"
  else
    return _type
  end
end

#################################################
def isStdType(_type)
 return _type == "string" || _type == "int" || _type == "double" ||
   _type == "float" || _type == "bool" || _type == "char" ||
   _type == "unsigned int"
end

#################################################
def printElem(_file, _spaces, _elem)

  # this currently short-circuits the plugin.sdf copy_data element.
  if _elem.attributes["name"].nil?
    _file.printf("%*s<xsd:sequence>\n", _spaces-2, "")
    _file.printf("%*s<xsd:any minOccurs='0' maxOccurs='unbounded' processContents='lax'/>\n", _spaces, "")
    _file.printf("%*s</xsd:sequence>\n", _spaces-2, "")
    return
  end

  type = _elem.attributes["type"]
  if isStdType(type)
    type = "xsd:" + xsdType(type)
  end

  minOccurs = '0'
  maxOccurs = 'unbounded'
  if _elem.attributes["required"] == '0'
    minOccurs='0'
    maxOccurs='1'
  elsif _elem.attributes["required"] == '1'
    minOccurs='1'
    maxOccurs='1'
  elsif _elem.attributes["required"] == '+'
    minOccurs='1'
    maxOccurs='unbounded'
  elsif _elem.attributes["required"] == '*'
    minOccurs='0'
    maxOccurs='unbounded'
  end

  _file.printf("%*s<xsd:choice  minOccurs='%s' maxOccurs='%s'>\n",
               _spaces, "", minOccurs, maxOccurs)

  # Print the complex type with a name
  if type.nil? || type == ""

    _file.printf("%*s<xsd:element name='%s'>\n",
                 _spaces, "", _elem.attributes["name"])

    if !_elem.elements["description"].nil? &&
       !_elem.elements["description"].text.nil?
      printDocumentation(_file, _spaces+2, _elem.elements["description"].text)
    end

    _file.printf("%*s<xsd:complexType>\n", _spaces+2, "")

    _file.printf("%*s<xsd:choice maxOccurs='unbounded'>\n", _spaces+4, "")

    _elem.get_elements("element").each do |elem|
      printElem(_file, _spaces+6, elem)
    end

    _elem.get_elements("include").each do |inc|
      printInclude(_file, _spaces+6, inc)
    end

    _file.printf("%*s</xsd:choice>\n", _spaces+4, "")

    # Print the attributes for the complex type
    # Attributes must go at the end of the complex type.
    _elem.get_elements("attribute").each do |attr|
      printAttribute(_file, _spaces+4, attr);
    end

    _file.printf("%*s</xsd:complexType>\n", _spaces+2, "")
  else

    attributes = _elem.get_elements("attribute")
    hasAttributes = attributes.size > 0

    if hasAttributes
      _file.printf("%*s<xsd:element name='%s'>\n",
                    _spaces, "", _elem.attributes["name"])
    else
      _file.printf("%*s<xsd:element name='%s' type='%s'>\n",
                    _spaces, "", _elem.attributes["name"], type)
    end

    if !_elem.elements["description"].nil? &&
       !_elem.elements["description"].text.nil?
      printDocumentation(_file, _spaces+2, _elem.elements["description"].text)
    end

    if hasAttributes
      _file.printf("%*s<xsd:complexType>\n", _spaces+2, "")
      _file.printf("%*s<xsd:simpleContent>\n", _spaces+4, "")
      _file.printf("%*s<xsd:extension base='%s'>\n", _spaces+6, "", type)

      # Print the attributes for the complex type
      # Attributes must go at the end of the complex type.
      attributes.each do |attr|
        printAttribute(_file, _spaces+8, attr);
      end

      _file.printf("%*s</xsd:extension>\n", _spaces+6, "")
      _file.printf("%*s</xsd:simpleContent>\n", _spaces+4, "")
      _file.printf("%*s</xsd:complexType>\n", _spaces+2, "")
    end

  end

  _file.printf("%*s</xsd:element>\n", _spaces, "")
  _file.printf("%*s</xsd:choice>\n", _spaces, "")
end

#################################################
def printDocumentation(_file, _spaces, _doc)
  _file.printf("%*s<xsd:annotation>\n", _spaces, "")

  _spaces += 2
  _file.printf("%*s<xsd:documentation xml:lang='en'>\n", _spaces, "")

  _spaces += 2
  _file.printf("%*s<![CDATA[%s]]>\n",_spaces, "", _doc);
  _spaces -= 2

  _file.printf("%*s</xsd:documentation>\n", _spaces, "")
  _spaces -= 2

  _file.printf("%*s</xsd:annotation>\n", _spaces, "")
end

#################################################
# Prints XSD contents of <include>
def printInclude(_file, _spaces, _attr)
  loc = $path + "/" + _attr.attributes['filename']
  doc = REXML::Document.new File.new(loc)

  doc.elements.each_with_index("element") do |elem, i|
    printElem(_file, _spaces+2, elem)
  end
end

#################################################
def printAttribute(_file, _spaces, _attr)
  name = _attr.attributes["name"]
  type = _attr.attributes["type"]
  use = ""
  default = ""

  if !_attr.attributes["required"].nil?
    if _attr.attributes["required"] == "1"
      use = "use='required'"
    elsif _attr.attributes["required"] == "0"
      use = "use='optional'"

      # Default is only valid if use is optional
      if !_attr.attributes["default"].nil?
        default="default='#{_attr.attributes["default"]}'"
      end
    end
  end

  if isStdType(type)
    type = "xsd:" + xsdType(type)
  end

  _file.printf("%*s<xsd:attribute name='%s' type='%s' %s %s>\n", _spaces,
               "", name, type, use, default)

  if !_attr.elements["description"].nil? &&
     !_attr.elements["description"].text.nil?
    printDocumentation(_file, _spaces+2, _attr.elements["description"].text)
  end
  _file.printf("%*s</xsd:attribute>\n", _spaces, "")
end

#################################################
# \brief Print the complete schema for an element into a file.
# \param[in] _file File pointer in which to print the schema.
# \param[in] _spaces Number of spaces to prepend to each line.
# \param[in] _elem The SDF element to convert to an xml schema.
def printXSD(_file, _spaces, _elem)

  if _elem.get_elements("element").size > 0 ||
     _elem.get_elements("attribute").size > 0 ||
     _elem.get_elements("include").size > 0

    # Print the complex type with a name
    _file.printf("%*s<xsd:element name='%s'>\n", _spaces, "",
                 _elem.attributes["name"])

    if !_elem.elements["description"].nil? &&
      !_elem.elements["description"].text.nil?
     printDocumentation(_file, _spaces+2, _elem.elements["description"].text)
    end

    _file.printf("%*s<xsd:complexType>\n", _spaces+2, "")

    if _elem.attributes['name'] != "plugin" &&
      (_elem.get_elements("element").size > 0 ||
       _elem.get_elements("include").size > 0)
      _file.printf("%*s<xsd:choice maxOccurs='unbounded'>\n", _spaces+4, "")
    end

    # Print all the child elements
    _elem.get_elements("element").each do |elem|
      printElem(_file, _spaces+6, elem);
    end

    # print XSD of <include> element
    _elem.get_elements("include").each do |inc|
      printInclude(_file, _spaces+4, inc);
    end

    if _elem.attributes['name'] != "plugin" &&
      (_elem.get_elements("element").size > 0 ||
       _elem.get_elements("include").size > 0)
      _file.printf("%*s</xsd:choice>\n", _spaces+4, "")
    end

    # Print the attributes for the complex type
    # Attributes must go at the end of the complex type.
    _elem.get_elements("attribute").each do |attr|
      printAttribute(_file, _spaces+4, attr);
    end

    # Close the complex type
    _file.printf("%*s</xsd:complexType>\n", _spaces+2, "")
    _file.printf("%*s</xsd:element>\n", _spaces, "")
  else
    type = _elem.attributes["type"]

    if isStdType(type)
      type = "xsd:" + type
    end

    if !type.nil?
      type = "type='" + type + "'"
    end

    _file.printf("%*s<xsd:element name='%s' %s/>\n", _spaces, "",
                 _elem.attributes["name"], type)
  end
end


infile = nil
outdir = nil

opt_parser = OptionParser.new do |o|
  o.on("-i", "--in [path]", String,
       "SDF file to compile") {|path| infile = path}
  o.on("-o", "--out [path]", String,
       "Output directory for source and header files") {|path| outdir = path}
  o.on("-s", "--sdf [path]", String,
       "Directory containing all the SDF files") {|path| $path = path}
  o.on("-h", "--help", "Display this help message") do
    puts opt_parser
    exit
  end
end
opt_parser.parse!

if infile.nil?
  puts "Missing option -i."
  exit
elsif !File.exists?(infile)
  puts "Input file[#{infile}] does not exist\n"
  exit
end

if $path.nil?
  puts "Missing option -s."
  exit
elsif !Dir.exists?($path)
  puts "SDF source dir[#{$path}] does not exist\n"
  exit
end

if outdir.nil?
  puts "Missing output directory, option -o."
  exit
elsif !Dir.exists?(outdir)
  Dir.mkdir(outdir)
end

doc = REXML::Document.new File.new(infile)

spaces = 2
doc.elements.each_with_index("element") do |elem, i|
  out_xsd = infile.split("/").last.sub("\.sdf","\.xsd")
  file = File.open(File.join(outdir, out_xsd), "w")

  file.print("<?xml version='1.0' encoding='UTF-8'?>\n")
  file.print("<xsd:schema xmlns:xsd='http://www.w3.org/2001/XMLSchema'>\n")

  # drop xml header
  File.readlines("#{$path}/schema/types.xsd").drop(1).each do |line|
    # skip root <schema> element
    if !line.include? "xsd:schema"
      file.printf("%*s#{line}", spaces-2, "")
    end
  end

  printXSD(file, spaces, elem)

  file.print("</xsd:schema>\n")
  file.close()
end
