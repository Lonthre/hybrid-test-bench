class_name HybridTestBench
extends Node3D


var shiny_steel = preload("res://materials/shiny_steel.tres")
var base_colour = null
@onready var pivot: Marker3D = $Marker3D
const SPECIMEN_LENGTH = 603
var pivot_z_offset: float

func _ready() -> void:
	base_colour = shiny_steel.albedo_color
	pivot_z_offset = pivot.position.z
	
func set_actuator_on():
	shiny_steel.albedo_color = Color.MEDIUM_SPRING_GREEN

func set_actuator_off():
	shiny_steel.albedo_color = Color.INDIAN_RED

func set_bending(vertical: float, horizontal:  float):
	pivot.rotation_degrees.x = (rad_to_deg(tan(vertical / SPECIMEN_LENGTH)))	
	pivot.position.z = horizontal / 1000 + pivot_z_offset
