import pygame

font = pygame.font.SysFont("Consolas", 18)

class Text:
    def __init__(self, text, x, y):
        self.text = text
        self.padding_x = 4
        self.padding_y = 1
        self.x = x
        self.y = y
    
    def draw(self, screen):
        text = font.render(self.text, True, (0, 255, 000))
        surface = pygame.Surface((text.get_width() + self.padding_x * 2, text.get_height() + self.padding_y * 2))
        surface.fill((0, 0, 0))
        surface.set_alpha(80)
        screen.blit(surface, pygame.Rect(self.x, self.y, surface.get_width(), surface.get_height()))
        screen.blit(text, pygame.Rect(self.padding_x + self.x, self.padding_y + self.y, text.get_width(), text.get_height()))