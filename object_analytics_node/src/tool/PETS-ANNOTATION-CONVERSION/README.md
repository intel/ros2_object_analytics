# This tool is used to convert PETS annotations(xml) to yml format which used by visualize tool in this module
#Eg. You can download PETS2009-S2L1-1.xml, PETS2009-S2L1.xml from internet and use this tool convert to YAML file

# PETS-ANNOTATION-CONVERSION
#PETS-ANNOTATION-CONVERSION

# validate XSD for XML files
#xmllint --noout --schema  pets_det.xsd PETS2009-S2L1.xml

# Convert XML to YAML file
#xml2yaml --schema pets_det.xsd --xml PETS2009-S2L1-c1-det.xml -o PETS2009-S2L1-c1-det.yml

#sed -i "1i%YAML:1.0" PETS2009-S2L1-c1-det.yml
