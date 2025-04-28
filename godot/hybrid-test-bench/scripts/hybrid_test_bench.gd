class_name HybridTestBench
extends Node3D


var shiny_steel = preload("res://materials/shiny_steel.tres")
var base_colour = null
@onready var indicator: CSGSphere3D = $CSGSphere3D


func _ready() -> void:
	base_colour = shiny_steel.albedo_color

func set_actuator_on():
	shiny_steel.albedo_color = Color.MEDIUM_SPRING_GREEN

func set_actuator_off():
	shiny_steel.albedo_color = Color.INDIAN_RED

func move_indicator(x, y):
	indicator.position.x = x
	indicator.position.y = y
