import math
import time
import torch
import torch.nn.functional as F



def bin_focal_loss(pred, target, gamma=2, alpha=0.6):
    """
    Focal loss based on binary cross entropy

    Increases loss for hard-to-classify samples
    Increases loss for negative samples
    
    pred:   (N, C, H, W)
    target: (N, C, H, W)
    width: grasp width

    For grasp points and grasp angles, higher confidence leads to larger alpha. When confidence=0, alpha=0.4; when confidence=1, alpha=0.6. 
    For grasp width, when y=0, alpha=0.4, otherwise alpha=0.6
    """
    n, c, h, w = pred.size()

    _loss = -1 * target * torch.log(pred + 1e-7) - (1 - target) * torch.log(1 - pred+1e-7)    # (N, C, H, W)
    _gamma = torch.abs(pred - target) ** gamma

    zeros_loc = torch.where(target == 0)
    _alpha = torch.ones_like(pred) * alpha
    _alpha[zeros_loc] = 1 - alpha

    loss = _loss * _gamma * _alpha
    loss = loss.sum() / (n*c*h*w)
    return loss


def focal_loss(net, x, y_pos, y_cos, y_sin, y_wid):
    """
    Calculate focal loss
    params:
    net: network
    x:     network input image   (batch, 1,   h, w)
    y_pos: grasp point label map   (batch, 1,   h, w)
    y_cos: grasp cos label map   (batch, 1,   h, w)
    y_sin: grasp sin label map   (batch, 1,   h, w)
    y_wid: grasp width label map   (batch, 1,   h, w)
    """

    # Get network predictions
    pred_pos, pred_cos, pred_sin, pred_wid = net(x)         # shape as the above

    pred_pos = torch.sigmoid(pred_pos)
    loss_pos = bin_focal_loss(pred_pos, y_pos, alpha=0.9) * 10

    pred_cos = torch.sigmoid(pred_cos)
    loss_cos = bin_focal_loss(pred_cos, (y_cos+1)/2, alpha=0.9) * 10

    pred_sin = torch.sigmoid(pred_sin)
    loss_sin = bin_focal_loss(pred_sin, (y_sin+1)/2, alpha=0.9) * 10

    pred_wid = torch.sigmoid(pred_wid)
    loss_wid = bin_focal_loss(pred_wid, y_wid, alpha=0.9) * 10


    return {
        'loss': loss_pos + loss_cos + loss_sin + loss_wid,
        'losses': {
            'loss_pos': loss_pos,
            'loss_cos': loss_cos,
            'loss_sin': loss_sin,
            'loss_wid': loss_wid,
        },
        'pred': {
            'pred_pos': pred_pos, 
            'pred_cos': pred_cos, 
            'pred_sin': pred_sin, 
            'pred_wid': pred_wid, 
        }
    }



def get_pred(net, xc):
    with torch.no_grad():
        pred_pos, pred_cos, pred_sin, pred_wid = net(xc)
        
        pred_pos = torch.sigmoid(pred_pos)
        pred_cos = torch.sigmoid(pred_cos)
        pred_sin = torch.sigmoid(pred_sin)
        pred_wid = torch.sigmoid(pred_wid)

    return pred_pos, pred_cos, pred_sin, pred_wid
